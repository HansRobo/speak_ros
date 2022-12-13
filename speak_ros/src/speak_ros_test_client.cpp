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

class TestClient : public rclcpp::Node
{
public:
  using Speak = speak_ros_interfaces::action::Speak;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Speak>;

  explicit TestClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("test_client", node_options)
  {
    client = rclcpp_action::create_client<Speak>(
      get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "/speak");
  }

  void sendGoal(std::string text)
  {
    using namespace std::placeholders;

    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Speak::Goal();
    goal_msg.text = text;
    goal_msg.speed_rate = 1.0;

    RCLCPP_INFO(get_logger(), "Sending goal");

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
      case Speak::Feedback::GENERATING:
        RCLCPP_INFO(get_logger(), "Generating sound file...");
        break;
      case Speak::Feedback::PLAYING:
        RCLCPP_INFO(get_logger(), "Playing sound file...");
        break;
      default:
        RCLCPP_INFO(get_logger(), "Unknown state");
        break;
    }
  }

  void resultCallback(const GoalHandle::WrappedResult & result)
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "elapsed time[s] : " << rclcpp::Duration(result.result->elapsed_time).seconds());
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  for (int i = 0; i < argc; i++) {
    std::cout << argv[i] << std::endl;
  }
  rclcpp::init(argc, argv);

  TestClient test_client;

  std::string input_text = "Hello World!";
  if (argc > 1) {
    input_text = argv[1];
  }
  test_client.sendGoal(input_text);

  rclcpp::spin(test_client.get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
