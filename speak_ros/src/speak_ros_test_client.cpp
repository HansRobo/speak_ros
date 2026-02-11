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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <speak_ros_interfaces/action/speak.hpp>

#include "speak_ros/parameter_validator.hpp"

class TestClient : public rclcpp::Node
{
public:
  using Speak = speak_ros_interfaces::action::Speak;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Speak>;
  using ParameterOverride = speak_ros_interfaces::msg::ParameterOverride;

  explicit TestClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("test_client", node_options)
  {
    client = rclcpp_action::create_client<Speak>(
      get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "/speak");
  }

  void sendGoal(std::string text, const std::vector<ParameterOverride> & parameters = {})
  {
    using namespace std::placeholders;

    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Speak::Goal();
    goal_msg.text = text;
    goal_msg.parameters = parameters;

    RCLCPP_INFO(get_logger(), "Sending goal with %zu parameters", parameters.size());

    auto send_goal_options = rclcpp_action::Client<Speak>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TestClient::goalResponseCallback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TestClient::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TestClient::resultCallback, this, _1);
    client->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Speak>::SharedPtr client;

  void goalResponseCallback(GoalHandle::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedbackCallback(
    GoalHandle::SharedPtr, const std::shared_ptr<const Speak::Feedback> feedback)
  {
    switch (feedback->state) {
      case Speak::Feedback::SYNTHESIZING:
        RCLCPP_INFO(
          get_logger(), "SYNTHESIZING (progress: %.1f%%)", feedback->synthesis_progress * 100.0);
        break;
      case Speak::Feedback::BUFFERING:
        RCLCPP_INFO(get_logger(), "BUFFERING (buffer: %.1f%%)", feedback->buffer_level * 100.0);
        break;
      case Speak::Feedback::PLAYING:
        RCLCPP_INFO(
          get_logger(), "PLAYING (%.2fs, buffer: %.1f%%)", feedback->playback_seconds,
          feedback->buffer_level * 100.0);
        break;
      default:
        RCLCPP_INFO(get_logger(), "Unknown state");
        break;
    }
  }

  void resultCallback(const GoalHandle::WrappedResult & result)
  {
    const char * termination_str = "UNKNOWN";
    switch (result.result->termination_reason) {
      case Speak::Result::COMPLETED:
        termination_str = "COMPLETED";
        break;
      case Speak::Result::CANCELLED:
        termination_str = "CANCELLED";
        break;
      case Speak::Result::ERROR:
        termination_str = "ERROR";
        break;
    }

    RCLCPP_INFO(
      get_logger(), "Result: %s, elapsed: %.2fs, samples: %u", termination_str,
      rclcpp::Duration(result.result->elapsed_time).seconds(), result.result->total_samples_played);
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestClient>();

  // Parse command line arguments
  std::string input_text = "Hello World!";
  std::vector<speak_ros_interfaces::msg::ParameterOverride> parameters;
  bool show_schema = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--param" && i + 1 < argc) {
      // Format: --param name=value
      std::string param = argv[++i];
      size_t eq_pos = param.find('=');
      if (eq_pos != std::string::npos) {
        speak_ros_interfaces::msg::ParameterOverride override;
        override.name = param.substr(0, eq_pos);
        override.value = param.substr(eq_pos + 1);
        parameters.push_back(override);
      } else {
        RCLCPP_ERROR(
          node->get_logger(), "Invalid parameter format: '%s'. Use --param name=value",
          param.c_str());
        return 1;
      }
    } else if (arg == "--show-schema") {
      show_schema = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [TEXT] [OPTIONS]\n"
                << "\n"
                << "Options:\n"
                << "  --param name=value  Set parameter override\n"
                << "  --show-schema       Show available parameters and exit\n"
                << "  --help, -h          Show this help message\n"
                << "\n"
                << "Examples:\n"
                << "  " << argv[0] << " \"Hello\"\n"
                << "  " << argv[0] << " \"Hello\" --param speaker=3 --param speedScale=1.5\n"
                << "  " << argv[0] << " --show-schema\n";
      return 0;
    } else {
      input_text = arg;
    }
  }

  // Create ParameterValidator and load schema
  auto validator = speak_ros::ParameterValidator::create(node);
  if (!validator->loadSchema("/get_parameter_schema", std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load parameter schema");
    return 1;
  }

  // Schema display mode
  if (show_schema) {
    validator->printSchema();
    return 0;
  }

  // Parameter validation
  if (!parameters.empty()) {
    auto result = validator->validate(parameters);
    if (!result.valid) {
      RCLCPP_ERROR(node->get_logger(), "Parameter validation failed:");
      for (const auto & error : result.errors) {
        RCLCPP_ERROR(node->get_logger(), "  - %s", error.c_str());
      }
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Parameter validation succeeded");
  }

  // Send goal
  node->sendGoal(input_text, parameters);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
