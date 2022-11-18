
#include <speak_ros/speak_ros.hpp>

namespace speak_ros
{
  SpeakROS::SpeakROS() : Node("speak_ros")
{
    RCLCPP_INFO(get_logger(), "Initializing SpeakROS...");
}

}  // end namespace speak_ros
