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

#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "speak_ros/speak_ros.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  pluginlib::ClassLoader<speak_ros::SpeakROSPluginBase> class_loader(
    "speak_ros", "speak_ros::SpeakROSPluginBase");

  auto found_libraries = class_loader.getDeclaredClasses();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("list_plugins"), "Found " << found_libraries.size() << " plugins");
  for (auto lib : found_libraries) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("list_plugins"), "plugin : " << lib);
  }

  rclcpp::shutdown();
  return 0;
}
