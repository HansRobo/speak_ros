// Copyright 2026 Kotaro Yoshimoto All rights reserved.
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

#ifndef SPEAK_ROS__PARAMETER_VALIDATOR_HPP_
#define SPEAK_ROS__PARAMETER_VALIDATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "speak_ros_interfaces/msg/parameter_override.hpp"
#include "speak_ros_interfaces/msg/parameter_schema.hpp"
#include "speak_ros_interfaces/srv/get_parameter_schema.hpp"

namespace speak_ros
{

/**
 * @brief Parameter validation result
 */
struct ValidationResult
{
  bool valid;                       // Whether validation succeeded
  std::vector<std::string> errors;  // List of error messages

  operator bool() const { return valid; }
};

/**
 * @brief Helper class for client-side parameter validation
 *
 * Usage example:
 * @code
 * auto validator = speak_ros::ParameterValidator::create(node);
 * validator->loadSchema("/get_parameter_schema", std::chrono::seconds(5));
 *
 * std::vector<ParameterOverride> overrides = {
 *   {.name = "speaker", .value = "3"},
 *   {.name = "speedScale", .value = "1.5"}
 * };
 * auto result = validator->validate(overrides);
 *
 * if (!result) {
 *   for (const auto& error : result.errors) {
 *     RCLCPP_ERROR(node->get_logger(), "%s", error.c_str());
 *   }
 * }
 * @endcode
 */
class ParameterValidator
{
public:
  using GetParameterSchema = speak_ros_interfaces::srv::GetParameterSchema;
  using ParameterOverride = speak_ros_interfaces::msg::ParameterOverride;
  using ParameterSchema = speak_ros_interfaces::msg::ParameterSchema;

  /**
   * @brief Create a ParameterValidator
   * @param node ROS 2 node
   * @return shared_ptr to ParameterValidator
   */
  static std::shared_ptr<ParameterValidator> create(rclcpp::Node::SharedPtr node)
  {
    return std::make_shared<ParameterValidator>(node);
  }

  /**
   * @brief Constructor
   * @param node ROS 2 node
   */
  explicit ParameterValidator(rclcpp::Node::SharedPtr node) : node_(node) {}

  /**
   * @brief Load parameter schema from service
   * @param service_name Service name (default: "/get_parameter_schema")
   * @param timeout Timeout duration
   * @return Whether schema loading succeeded
   */
  bool loadSchema(
    const std::string & service_name = "/get_parameter_schema",
    std::chrono::seconds timeout = std::chrono::seconds(5))
  {
    auto client = node_->create_client<GetParameterSchema>(service_name);

    if (!client->wait_for_service(timeout)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Service '%s' not available after waiting", service_name.c_str());
      return false;
    }

    auto request = std::make_shared<GetParameterSchema::Request>();
    auto future = client->async_send_request(request);

    if (
      rclcpp::spin_until_future_complete(node_, future, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service '%s'", service_name.c_str());
      return false;
    }

    auto response = future.get();

    // Save schema
    schema_ = response->parameters;

    // Build name -> index map
    param_index_.clear();
    for (size_t i = 0; i < schema_.size(); ++i) {
      param_index_[schema_[i].name] = i;
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded parameter schema with %zu parameters", schema_.size());

    return true;
  }

  /**
   * @brief Validate parameters
   * @param overrides List of parameter overrides
   * @return Validation result
   */
  ValidationResult validate(const std::vector<ParameterOverride> & overrides) const
  {
    ValidationResult result;
    result.valid = true;

    // Check if schema is loaded
    if (schema_.empty()) {
      result.valid = false;
      result.errors.push_back("Parameter schema not loaded. Call loadSchema() first.");
      return result;
    }

    // Check each parameter
    for (const auto & override : overrides) {
      const auto & name = override.name;
      const auto & value = override.value;

      // Check parameter name existence
      auto it = param_index_.find(name);
      if (it == param_index_.end()) {
        result.valid = false;
        result.errors.push_back("Unknown parameter: '" + name + "'" + getSuggestion(name));
        continue;
      }

      size_t idx = it->second;
      const auto & type = schema_[idx].type;

      // Check type conversion validity
      if (type == "int") {
        try {
          std::stoi(value);
        } catch (const std::exception & e) {
          result.valid = false;
          result.errors.push_back(
            "Parameter '" + name + "' expects type 'int', but got invalid value: '" + value + "'");
        }
      } else if (type == "double") {
        try {
          std::stod(value);
        } catch (const std::exception & e) {
          result.valid = false;
          result.errors.push_back(
            "Parameter '" + name + "' expects type 'double', but got invalid value: '" + value +
            "'");
        }
      }
      // string type is always valid
    }

    return result;
  }

  /**
   * @brief Get list of available parameters
   * @return List of parameter names
   */
  std::vector<std::string> getParameterNames() const
  {
    std::vector<std::string> names;
    for (const auto & param : schema_) {
      names.push_back(param.name);
    }
    return names;
  }

  /**
   * @brief Display detailed parameter information
   */
  void printSchema() const
  {
    if (schema_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "No schema loaded");
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "Available parameters:");
    for (const auto & param : schema_) {
      RCLCPP_INFO(
        node_->get_logger(), "  - %s (%s): %s [default: %s]", param.name.c_str(),
        param.type.c_str(), param.description.c_str(), param.default_value.c_str());
    }
  }

private:
  /**
   * @brief Suggest similar parameter for invalid parameter name
   * @param name Invalid parameter name
   * @return Suggestion message (empty string if no suggestion)
   */
  std::string getSuggestion(const std::string & name) const
  {
    // Calculate similarity using simple Levenshtein distance
    std::string best_match;
    int min_distance = std::numeric_limits<int>::max();

    for (const auto & param : schema_) {
      int dist = levenshteinDistance(name, param.name);
      if (dist < min_distance) {
        min_distance = dist;
        best_match = param.name;
      }
    }

    // Suggest if distance is 3 or less
    if (min_distance <= 3 && !best_match.empty()) {
      return ". Did you mean '" + best_match + "'?";
    }

    return "";
  }

  /**
   * @brief Calculate Levenshtein distance
   */
  static int levenshteinDistance(const std::string & s1, const std::string & s2)
  {
    const size_t len1 = s1.size(), len2 = s2.size();
    std::vector<std::vector<int>> d(len1 + 1, std::vector<int>(len2 + 1));

    for (size_t i = 0; i <= len1; ++i) d[i][0] = i;
    for (size_t j = 0; j <= len2; ++j) d[0][j] = j;

    for (size_t i = 1; i <= len1; ++i) {
      for (size_t j = 1; j <= len2; ++j) {
        int cost = (s1[i - 1] == s2[j - 1]) ? 0 : 1;
        d[i][j] = std::min({
          d[i - 1][j] + 1,        // deletion
          d[i][j - 1] + 1,        // insertion
          d[i - 1][j - 1] + cost  // substitution
        });
      }
    }

    return d[len1][len2];
  }

  rclcpp::Node::SharedPtr node_;

  // Schema information
  std::vector<ParameterSchema> schema_;

  // Map for fast lookup
  std::unordered_map<std::string, size_t> param_index_;
};

}  // namespace speak_ros

#endif  // SPEAK_ROS__PARAMETER_VALIDATOR_HPP_
