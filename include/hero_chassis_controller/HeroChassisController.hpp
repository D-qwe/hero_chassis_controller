#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// ros_control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

// control_toolbox (选做4核心)
#include <control_toolbox/pid.h>

// dynamic_reconfigure (选做4调参核心)
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/PidConfig.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  // 控制器初始化函数
  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  // 控制器核心循环函数
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  // 接收速度指令的回调函数
  void cmdVelCallback(const geometry_msgs::Twist& msg);

  // 动态调参回调函数
  void reconfigCallback(hero_chassis_controller::PidConfig& config, uint32_t level);

  // 四个轮子的句柄
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

  // 订阅者
  ros::Subscriber cmd_vel_sub_;

  // 存储当前接收到的速度指令
  double target_vx_ = 0.0;
  double target_vy_ = 0.0;
  double target_wz_ = 0.0;

  // ====== 选做 4：PID 控制相关 ======
  control_toolbox::Pid pid_fl_;
  control_toolbox::Pid pid_fr_;
  control_toolbox::Pid pid_bl_;
  control_toolbox::Pid pid_br_;

  // 用于从配置文件读取的 PID 参数
  double p_, i_, d_, i_max_, i_min_;

  // 动态调参服务器指针
  std::shared_ptr<dynamic_reconfigure::Server<hero_chassis_controller::PidConfig>> dyn_server_;
};

}  // namespace hero_chassis_controller