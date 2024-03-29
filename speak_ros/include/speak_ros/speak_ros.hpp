// Copyright 2022 Kotaro Yoshimoto All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SPEAK_ROS__SPEAK_ROS_HPP_
#define SPEAK_ROS__SPEAK_ROS_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "speak_ros/speak_ros_plugin_base.hpp"
#include "speak_ros_interfaces/action/speak.hpp"

namespace speak_ros
{
class SpeakROS : public rclcpp::Node
{
public:
  explicit SpeakROS(rclcpp::NodeOptions options) : rclcpp::Node("speak_ros", options)
  {
    pluginlib::ClassLoader<speak_ros::SpeakROSPluginBase> class_loader(
      "speak_ros", "speak_ros::SpeakROSPluginBase");

    declare_parameter<std::string>("plugin_name", "open_jtalk_plugin::OpenJTalkPlugin");
    get_parameter("plugin_name", plugin_name);

    try {
      plugin = class_loader.createSharedInstance(plugin_name);
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Can't find plugin : " << plugin_name << " : " << ex.what());
      return;
    }

    plugin->setParameterInterface(get_node_parameters_interface());

    RCLCPP_INFO_STREAM(get_logger(), "Using plugin : " << plugin_name);

    plugin->updateParameters(true);

    using Speak = speak_ros_interfaces::action::Speak;
    server = rclcpp_action::create_server<Speak>(
      get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "speak",
      // handle goal
      [](const rclcpp_action::GoalUUID, std::shared_ptr<const Speak::Goal> goal)
        -> rclcpp_action::GoalResponse { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      // handle cancel
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Speak>> goal_handle)
        -> rclcpp_action::CancelResponse { return rclcpp_action::CancelResponse::REJECT; },
      // handle accepted
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Speak>> goal_handle) -> void {
        std::thread([goal_handle, this]() {
          const auto goal = goal_handle->get_goal();
          auto feedback = std::make_shared<Speak::Feedback>();

          feedback->state = Speak::Feedback::GENERATING;
          goal_handle->publish_feedback(feedback);

          generateSoundFile(goal->text);

          std::promise<void> play_finish_notifier;
          std::future<void> play_finish_monitor = play_finish_notifier.get_future();

          auto play_start_time = now();
          auto play_thread = std::thread([&play_finish_notifier, this]() {
            playSoundFile();
            play_finish_notifier.set_value();
          });

          using std::literals::chrono_literals::operator""s;

          for (rclcpp::FutureReturnCode return_code;
               return_code != rclcpp::FutureReturnCode::SUCCESS;
               return_code = rclcpp::spin_until_future_complete(
                 this->get_node_base_interface(), play_finish_monitor, 0.5s)) {
            if (return_code == rclcpp::FutureReturnCode::INTERRUPTED) {
              auto result = std::make_shared<Speak::Result>();
              result->elapsed_time = now() - play_start_time;
              goal_handle->abort(result);
              play_thread.join();
              return;
            }
            feedback->state = Speak::Feedback::PLAYING;
            goal_handle->publish_feedback(feedback);
          }

          play_thread.join();
          auto result = std::make_shared<Speak::Result>();
          result->elapsed_time = now() - play_start_time;
          goal_handle->succeed(result);
        }).detach();
      });
  }

  void generateSoundFile(std::string input_text)
  {
    std::filesystem::create_directory(base_directory);
    generated_sound_path = plugin->generateSoundFile(input_text, base_directory, "generated");
  }

  void playSoundFile() { plugin->playSoundFile(generated_sound_path); }

private:
  std::filesystem::path base_directory = "/tmp/speak_ros";
  std::filesystem::path generated_sound_path;
  std::shared_ptr<speak_ros::SpeakROSPluginBase> plugin = nullptr;
  rclcpp_action::Server<speak_ros_interfaces::action::Speak>::SharedPtr server;
  std::string plugin_name;
};

}  // namespace speak_ros

#endif  // SPEAK_ROS__SPEAK_ROS_HPP_
