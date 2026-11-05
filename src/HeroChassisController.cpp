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
    "cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  // 将接收到的速度指令存下来，供 update 使用
  target_vx_ = msg.linear.x;
  target_vy_ = msg.linear.y;
  target_wz_ = msg.angular.z;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
  // 【选做 5 核心】这里目前先用固定值占位，下一步我们会把逆运动学公式写在这里
  // 先让轮子有一点力，确保整个框架能跑通
  double temp_tau = 0.1;

  front_left_joint_.setCommand(temp_tau);
  front_right_joint_.setCommand(temp_tau);
  back_left_joint_.setCommand(temp_tau);
  back_right_joint_.setCommand(temp_tau);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)