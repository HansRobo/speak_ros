
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <speak_ros/speak_ros.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<speak_ros::SpeakROS>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
