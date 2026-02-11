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

#ifndef SPEAK_ROS__SPEAK_ROS_PLUGIN_HPP_
#define SPEAK_ROS__SPEAK_ROS_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "speak_ros/audio_types.hpp"

namespace speak_ros
{
struct Parameter
{
  std::string name;
  std::string description;
  std::variant<int, double, std::string> default_value;
};

class SpeakROSPluginBase
{
public:
  void setParameterInterface(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface)
  {
    this->parameter_interface = parameter_interface;
  }

  [[nodiscard]] virtual std::string getPluginName() const = 0;

  /**
   * @brief Get audio format output by the plugin
   * @return Audio format information
   */
  virtual AudioFormat getAudioFormat() const = 0;

  /**
   * @brief Synthesize text to speech and return via callback per chunk
   * @param text Text to synthesize
   * @param callback Callback to receive audio chunks (return false to request abort)
   * @param cancel_token Cancellation request token (abort if true)
   */
  virtual void synthesize(
    const std::string & text, AudioChunkCallback callback, CancelToken cancel_token) = 0;

  virtual std::vector<Parameter> getParametersDefault() const { return std::vector<Parameter>(); }
  virtual void importParameters(
    const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
  {
  }

  /**
   * @brief Declare parameters and load initial values (once at node startup)
   */
  void initializeParameters()
  {
    auto parameters_default = getParametersDefault();
    std::unordered_map<std::string, std::variant<int, double, std::string>> parameters;

    std::string topic_prefix = getPluginName() + "/";
    for (const auto & parameter : parameters_default) {
      if (std::holds_alternative<int>(parameter.default_value)) {
        parameter_interface->declare_parameter(
          topic_prefix + parameter.name,
          rclcpp::ParameterValue(std::get<int>(parameter.default_value)));
        parameters[parameter.name] =
          (int)parameter_interface->get_parameter(topic_prefix + parameter.name).as_int();
      } else if (std::holds_alternative<double>(parameter.default_value)) {
        parameter_interface->declare_parameter(
          topic_prefix + parameter.name,
          rclcpp::ParameterValue(std::get<double>(parameter.default_value)));
        parameters[parameter.name] =
          parameter_interface->get_parameter(topic_prefix + parameter.name).as_double();
      } else if (std::holds_alternative<std::string>(parameter.default_value)) {
        parameter_interface->declare_parameter(
          topic_prefix + parameter.name,
          rclcpp::ParameterValue(std::get<std::string>(parameter.default_value)));
        parameters[parameter.name] =
          parameter_interface->get_parameter(topic_prefix + parameter.name).as_string();
      }
    }
    importParameters(parameters);
  }

  /**
   * @brief Load ROS parameters and merge Goal overrides (per request)
   * @param overrides List of parameters to override (name-value pairs)
   */
  void loadParametersWithOverrides(
    const std::vector<std::pair<std::string, std::string>> & overrides = {})
  {
    auto parameters_default = getParametersDefault();
    std::unordered_map<std::string, std::variant<int, double, std::string>> parameters;

    std::string topic_prefix = getPluginName() + "/";

    // Step 1: Load from ROS parameters
    for (const auto & parameter : parameters_default) {
      if (std::holds_alternative<int>(parameter.default_value)) {
        parameters[parameter.name] =
          (int)parameter_interface->get_parameter(topic_prefix + parameter.name).as_int();
      } else if (std::holds_alternative<double>(parameter.default_value)) {
        parameters[parameter.name] =
          parameter_interface->get_parameter(topic_prefix + parameter.name).as_double();
      } else if (std::holds_alternative<std::string>(parameter.default_value)) {
        parameters[parameter.name] =
          parameter_interface->get_parameter(topic_prefix + parameter.name).as_string();
      }
    }

    // Step 2: Merge Goal overrides
    for (const auto & [name, value_str] : overrides) {
      // Search parameter definition
      auto it = std::find_if(
        parameters_default.begin(), parameters_default.end(),
        [&name](const Parameter & p) { return p.name == name; });

      if (it == parameters_default.end()) {
        RCLCPP_WARN(
          rclcpp::get_logger("speak_ros_plugin"), "Unknown parameter override: '%s', ignoring",
          name.c_str());
        continue;
      }

      // Convert according to default_value type
      try {
        if (std::holds_alternative<int>(it->default_value)) {
          parameters[name] = std::stoi(value_str);
        } else if (std::holds_alternative<double>(it->default_value)) {
          parameters[name] = std::stod(value_str);
        } else if (std::holds_alternative<std::string>(it->default_value)) {
          parameters[name] = value_str;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger("speak_ros_plugin"),
          "Failed to convert parameter '%s' value '%s': %s, ignoring", name.c_str(),
          value_str.c_str(), e.what());
      }
    }

    // Step 3: importParameters(params)
    importParameters(parameters);
  }

protected:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface;
};
}  // namespace speak_ros

#endif  // SPEAK_ROS__SPEAK_ROS_PLUGIN_HPP_
