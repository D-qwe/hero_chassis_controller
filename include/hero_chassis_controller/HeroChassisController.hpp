#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ros_control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

// control_toolbox
#include <control_toolbox/pid.h>

// dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_controller/PidConfig.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  void cmdVelCallback(const geometry_msgs::Twist& msg);
  void reconfigCallback(hero_chassis_controller::PidConfig& config, uint32_t level);

  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
  ros::Subscriber cmd_vel_sub_;

  double target_vx_ = 0.0;
  double target_vy_ = 0.0;
  double target_wz_ = 0.0;

  control_toolbox::Pid pid_fl_;
  control_toolbox::Pid pid_fr_;
  control_toolbox::Pid pid_bl_;
  control_toolbox::Pid pid_br_;
  double p_, i_, d_, i_max_, i_min_;
  std::shared_ptr<dynamic_reconfigure::Server<hero_chassis_controller::PidConfig>> dyn_server_;

  ros::Publisher odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double vx_ = 0.0;
  double vy_ = 0.0;
  double wz_ = 0.0;
  ros::Time last_time_;

  // ====== 选做 7：世界坐标系控制 ======
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool use_world_frame_ = false; // 模式开关：默认关闭（底盘模式），开启后为世界模式
};

}  // namespace hero_chassis_controller