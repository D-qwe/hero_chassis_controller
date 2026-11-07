#include "hero_chassis_controller/HeroChassisController.hpp"
#include <algorithm>

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

  ROS_INFO("HeroChassisController initialized with dynamic reconfigure.");
  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  target_vx_ = msg.linear.x;
  target_vy_ = msg.linear.y;
  target_wz_ = msg.angular.z;
}

void HeroChassisController::reconfigCallback(hero_chassis_controller::PidConfig& config, uint32_t level) {
  // 当你在 rqt_reconfigure 拖动滑块时，这里会被触发
  p_ = config.p;
  i_ = config.i;
  d_ = config.d;
  i_max_ = config.i_max;
  i_min_ = config.i_min;

  // 更新四个轮子的 PID 参数
  pid_fl_.setGains(p_, i_, d_, i_max_, i_min_);
  pid_fr_.setGains(p_, i_, d_, i_max_, i_min_);
  pid_bl_.setGains(p_, i_, d_, i_max_, i_min_);
  pid_br_.setGains(p_, i_, d_, i_max_, i_min_);

  ROS_INFO_STREAM("PID Params Updated: p=" << p_ << " i=" << i_ << " d=" << d_);
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
  // 1. 物理参数
  double r = 0.07625;
  double L = 0.2;
  double W = 0.2;
  double L_plus_W = L + W;

  // 2. 逆运动学：计算期望角速度 (rad/s)
  double w_fl = (target_vx_ - target_vy_ - L_plus_W * target_wz_) / r;
  double w_fr = (target_vx_ + target_vy_ + L_plus_W * target_wz_) / r;
  double w_bl = (target_vx_ + target_vy_ - L_plus_W * target_wz_) / r;
  double w_br = (target_vx_ - target_vy_ + L_plus_W * target_wz_) / r;

  // 3. 获取实际角速度
  double real_w_fl = front_left_joint_.getVelocity();
  double real_w_fr = front_right_joint_.getVelocity();
  double real_w_bl = back_left_joint_.getVelocity();
  double real_w_br = back_right_joint_.getVelocity();

  // 4. PID 计算
  double tau_fl = pid_fl_.computeCommand(w_fl - real_w_fl, period);
  double tau_fr = pid_fr_.computeCommand(w_fr - real_w_fr, period);
  double tau_bl = pid_bl_.computeCommand(w_bl - real_w_bl, period);
  double tau_br = pid_br_.computeCommand(w_br - real_w_br, period);

  // 5. 力矩限幅，最大不超过 5.0，最小不低于 -5.0
  double max_tau = 5.0;
  tau_fl = std::max(-max_tau, std::min(max_tau, tau_fl));
  tau_fr = std::max(-max_tau, std::min(max_tau, tau_fr));
  tau_bl = std::max(-max_tau, std::min(max_tau, tau_bl));
  tau_br = std::max(-max_tau, std::min(max_tau, tau_br));

  // 6. 下发力矩
  front_left_joint_.setCommand(tau_fl);
  front_right_joint_.setCommand(tau_fr);
  back_left_joint_.setCommand(tau_bl);
  back_right_joint_.setCommand(tau_br);

  // 7. 打印日志 (每秒一次)
  ROS_INFO_STREAM_THROTTLE(1.0, "Vx: " << target_vx_ << " | Target FL: " << w_fl << " | Real FL: " << real_w_fl << " | Tau FL: " << tau_fl);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase);