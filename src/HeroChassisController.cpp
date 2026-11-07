#include "hero_chassis_controller/HeroChassisController.hpp"

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // 1. 获取四个轮子的句柄 (注意这里的名字要和 URDF 里完全一样)
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  // 2. 订阅 /cmd_vel 话题，接收速度指令
  cmd_vel_sub_ = controller_nh.subscribe(
    "/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  // 将接收到的速度指令存下来，供 update 使用
  target_vx_ = msg.linear.x;
  target_vy_ = msg.linear.y;
  target_wz_ = msg.angular.z;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
  // 1. 从 URDF 提取的物理参数
  double r = 0.07625;      // 轮子半径
  double L = 0.2;          // wheel_track / 2
  double W = 0.2;          // wheel_base / 2
  double L_plus_W = L + W; // 0.4

  // 2. 逆运动学：根据目标底盘速度计算四个轮子的期望角速度 (rad/s)
  double w_fl = (target_vx_ - target_vy_ - L_plus_W * target_wz_) / r;
  double w_fr = (target_vx_ + target_vy_ + L_plus_W * target_wz_) / r;
  double w_bl = (target_vx_ + target_vy_ - L_plus_W * target_wz_) / r;
  double w_br = (target_vx_ - target_vy_ + L_plus_W * target_wz_) / r;

  // 3. 获取轮子当前的真实速度
  double real_w_fl = front_left_joint_.getVelocity();
  double real_w_fr = front_right_joint_.getVelocity();
  double real_w_bl = back_left_joint_.getVelocity();
  double real_w_br = back_right_joint_.getVelocity();

  // 4. 简易 P 控制器 (比例控制)
  double Kp = 5.0;

  double tau_fl = Kp * (w_fl - real_w_fl);
  double tau_fr = Kp * (w_fr - real_w_fr);
  double tau_bl = Kp * (w_bl - real_w_bl);
  double tau_br = Kp * (w_br - real_w_br);

  // 5. 下发力矩
  front_left_joint_.setCommand(tau_fl);
  front_right_joint_.setCommand(tau_fr);
  back_left_joint_.setCommand(tau_bl);
  back_right_joint_.setCommand(tau_br);
  // 放在 update 函数的最后
  ROS_INFO_STREAM_THROTTLE(1.0, "Target Vx: " << target_vx_ << " | FL Real: " << real_w_fl << " | FL Tau: " << tau_fl);
}

}  // namespace hero_chassis_controller



PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase);