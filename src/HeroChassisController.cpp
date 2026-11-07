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

  // 4. 初始化里程计发布者 (选做 6 核心)
  odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

  last_time_ = ros::Time::now();

  ROS_INFO("HeroChassisController initialized with Odometry and TF.");
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
  double r = 0.07625;
  double L = 0.2;
  double W = 0.2;
  double L_plus_W = L + W;

  // --- 逆运动学：算期望速度 ---
  double w_fl = (target_vx_ - target_vy_ - L_plus_W * target_wz_) / r;
  double w_fr = (target_vx_ + target_vy_ + L_plus_W * target_wz_) / r;
  double w_bl = (target_vx_ + target_vy_ - L_plus_W * target_wz_) / r;
  double w_br = (target_vx_ - target_vy_ + L_plus_W * target_wz_) / r;

  // --- 获取实际速度 ---
  double real_w_fl = front_left_joint_.getVelocity();
  double real_w_fr = front_right_joint_.getVelocity();
  double real_w_bl = back_left_joint_.getVelocity();
  double real_w_br = back_right_joint_.getVelocity();

  // --- PID 计算与力矩限幅 ---
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

  // ================= 选做 6：正运动学与里程计 =================

  // 1. 正运动学公式（逆运动学的逆推）
  vx_ = (real_w_fl + real_w_fr + real_w_bl + real_w_br) * r / 4.0;
  vy_ = (-real_w_fl + real_w_fr + real_w_bl - real_w_br) * r / 4.0;
  wz_ = (-real_w_fl + real_w_fr - real_w_bl + real_w_br) * r / (4.0 * L_plus_W);

  // 2. 积分计算当前位置
  double dt = (time - last_time_).toSec();
  if (dt > 0.0) {
    // 将底盘速度转换到世界坐标系
    double vx_world = vx_ * std::cos(theta_) - vy_ * std::sin(theta_);
    double vy_world = vx_ * std::sin(theta_) + vy_ * std::cos(theta_);

    x_ += vx_world * dt;
    y_ += vy_world * dt;
    theta_ += wz_ * dt;
    last_time_ = time;
  }

  // 3. 发布 /odom 话题 (nav_msgs/Odometry)
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.angular.z = wz_;

  odom_pub_.publish(odom);

  // 4. 发布 TF 坐标变换 (odom -> base_link)
  geometry_msgs::TransformStamped t;
  t.header.stamp = time;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(t);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase);