
#pragma once

// C++
#include <memory>
#include <string>

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

namespace speak_ros
{
class SpeakROS : public rclcpp::Node
{
public:
  SpeakROS();

private:
};

}  // namespace speak_ros
