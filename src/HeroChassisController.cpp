#include "hero_chassis_controller/HeroChassisController.hpp"
#include <algorithm>
#include <cmath>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // 1. 获取四个轮子的句柄
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  // 2. 订阅 /cmd_vel 话题
  cmd_vel_sub_ = controller_nh.subscribe(
      "/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  // 3. 创建动态调参服务器
  dyn_server_ = std::make_shared<dynamic_reconfigure::Server<hero_chassis_controller::PidConfig>>(controller_nh);
  dynamic_reconfigure::Server<hero_chassis_controller::PidConfig>::CallbackType f;
  f = boost::bind(&HeroChassisController::reconfigCallback, this, _1, _2);
  dyn_server_->setCallback(f);

  // 4. 初始化里程计发布者 (选做 6)
  odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

  // 5. 初始化 TF 监听器 (选做 7)
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 6. 从配置读取是否使用世界坐标系 (默认 false 为底盘坐标系)
  controller_nh.param("use_world_frame", use_world_frame_, false);
  ROS_INFO_STREAM("World Frame Mode: " << (use_world_frame_ ? "ENABLED" : "DISABLED"));

  last_time_ = ros::Time::now();
  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  target_vx_ = msg.linear.x;
  target_vy_ = msg.linear.y;
  target_wz_ = msg.angular.z;
}

void HeroChassisController::reconfigCallback(hero_chassis_controller::PidConfig& config, uint32_t level) {
  p_ = config.p;
  i_ = config.i;
  d_ = config.d;
  i_max_ = config.i_max;
  i_min_ = config.i_min;

  pid_fl_.setGains(p_, i_, d_, i_max_, i_min_);
  pid_fr_.setGains(p_, i_, d_, i_max_, i_min_);
  pid_bl_.setGains(p_, i_, d_, i_max_, i_min_);
  pid_br_.setGains(p_, i_, d_, i_max_, i_min_);
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
  // 预设底盘坐标系速度（默认为接收到的速度）
  double cmd_vx = target_vx_;
  double cmd_vy = target_vy_;

  // ====== 选做 7：世界坐标系速度转换 ======
  if (use_world_frame_) {
    try {
      // 查找从 odom 到 base_link 的当前变换
      geometry_msgs::TransformStamped trans = tf_buffer_->lookupTransform("base_link", "odom", time, ros::Duration(0.05));

      // 提取旋转矩阵（四元数转旋转矩阵）
      tf2::Quaternion q(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // 将世界速度通过旋转矩阵转换到底盘坐标系
      cmd_vx = target_vx_ * cos(yaw) + target_vy_ * sin(yaw);
      cmd_vy = -target_vx_ * sin(yaw) + target_vy_ * cos(yaw);

    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0, "TF Lookup failed: %s", ex.what());
      // 如果 TF 查不到（比如刚启动时），暂时用车身积分角度代替
      cmd_vx = target_vx_ * cos(theta_) + target_vy_ * sin(theta_);
      cmd_vy = -target_vx_ * sin(theta_) + target_vy_ * cos(theta_);
    }
  }

  // ====== 逆运动学 ======
  double r = 0.07625;
  double L = 0.2;
  double W = 0.2;
  double L_plus_W = L + W;

  double w_fl = (cmd_vx - cmd_vy - L_plus_W * target_wz_) / r;
  double w_fr = (cmd_vx + cmd_vy + L_plus_W * target_wz_) / r;
  double w_bl = (cmd_vx + cmd_vy - L_plus_W * target_wz_) / r;
  double w_br = (cmd_vx - cmd_vy + L_plus_W * target_wz_) / r;

  // ====== PID 计算与限幅 ======
  double real_w_fl = front_left_joint_.getVelocity();
  double real_w_fr = front_right_joint_.getVelocity();
  double real_w_bl = back_left_joint_.getVelocity();
  double real_w_br = back_right_joint_.getVelocity();

  double tau_fl = pid_fl_.computeCommand(w_fl - real_w_fl, period);
  double tau_fr = pid_fr_.computeCommand(w_fr - real_w_fr, period);
  double tau_bl = pid_bl_.computeCommand(w_bl - real_w_bl, period);
  double tau_br = pid_br_.computeCommand(w_br - real_w_br, period);

  double max_tau = 5.0;
  tau_fl = std::max(-max_tau, std::min(max_tau, tau_fl));
  tau_fr = std::max(-max_tau, std::min(max_tau, tau_fr));
  tau_bl = std::max(-max_tau, std::min(max_tau, tau_bl));
  tau_br = std::max(-max_tau, std::min(max_tau, tau_br));

  front_left_joint_.setCommand(tau_fl);
  front_right_joint_.setCommand(tau_fr);
  back_left_joint_.setCommand(tau_bl);
  back_right_joint_.setCommand(tau_br);

  // ====== 选做 6：正运动学与里程计 ======
  vx_ = (real_w_fl + real_w_fr + real_w_bl + real_w_br) * r / 4.0;
  vy_ = (-real_w_fl + real_w_fr + real_w_bl - real_w_br) * r / 4.0;
  wz_ = (-real_w_fl + real_w_fr - real_w_bl + real_w_br) * r / (4.0 * L_plus_W);

  double dt = (time - last_time_).toSec();
  if (dt > 0.0) {
    double vx_world = vx_ * std::cos(theta_) - vy_ * std::sin(theta_);
    double vy_world = vx_ * std::sin(theta_) + vy_ * std::cos(theta_);
    x_ += vx_world * dt;
    y_ += vy_world * dt;
    theta_ += wz_ * dt;
    last_time_ = time;
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  tf2::Quaternion q_odom;
  q_odom.setRPY(0, 0, theta_);
  odom.pose.pose.orientation.x = q_odom.x();
  odom.pose.pose.orientation.y = q_odom.y();
  odom.pose.pose.orientation.z = q_odom.z();
  odom.pose.pose.orientation.w = q_odom.w();
  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.angular.z = wz_;
  odom_pub_.publish(odom);

  geometry_msgs::TransformStamped t;
  t.header.stamp = time;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.rotation.x = q_odom.x();
  t.transform.rotation.y = q_odom.y();
  t.transform.rotation.z = q_odom.z();
  t.transform.rotation.w = q_odom.w();
  tf_broadcaster_->sendTransform(t);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase);