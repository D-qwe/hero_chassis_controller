#include "hero_chassis_controller/HeroChassisController.hpp"

// STD
#include <string>

namespace hero_chassis_controller {

HeroChassisController::HeroChassisController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle) {
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &HeroChassisController::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &HeroChassisController::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

HeroChassisController::~HeroChassisController() {
}

bool HeroChassisController::readParameters() {
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void HeroChassisController::topicCallback(const sensor_msgs::Temperature &message) {
  algorithm_.addData(message.temperature);
}

bool HeroChassisController::serviceCallback(std_srvs::Trigger::Request &request,
                                         std_srvs::Trigger::Response &response) {
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

} /* namespace */
