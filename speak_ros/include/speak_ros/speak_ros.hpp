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

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "speak_ros/audio_player.hpp"
#include "speak_ros/audio_queue.hpp"
#include "speak_ros/audio_types.hpp"
#include "speak_ros/speak_ros_plugin_base.hpp"
#include "speak_ros_interfaces/action/speak.hpp"
#include "speak_ros_interfaces/srv/get_parameter_schema.hpp"

namespace speak_ros
{

class SpeakROS : public rclcpp::Node
{
public:
  using Speak = speak_ros_interfaces::action::Speak;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Speak>;
  using GetParameterSchema = speak_ros_interfaces::srv::GetParameterSchema;

  explicit SpeakROS(const rclcpp::NodeOptions & options);
  ~SpeakROS();

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Speak::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void executeGoal(std::shared_ptr<GoalHandle> goal_handle);

  void handleGetParameterSchema(
    const std::shared_ptr<GetParameterSchema::Request> request,
    std::shared_ptr<GetParameterSchema::Response> response);

  pluginlib::ClassLoader<SpeakROSPluginBase> class_loader_;
  std::shared_ptr<SpeakROSPluginBase> plugin_;
  std::string plugin_name_;

  rclcpp_action::Server<Speak>::SharedPtr action_server_;

  rclcpp::Service<GetParameterSchema>::SharedPtr parameter_schema_service_;

  AudioQueue audio_queue_;
  AudioPlayer audio_player_;

  CancelToken current_cancel_token_;
  std::mutex cancel_mutex_;

  std::thread execution_thread_;
};

}  // namespace speak_ros

#endif  // SPEAK_ROS__SPEAK_ROS_HPP_
