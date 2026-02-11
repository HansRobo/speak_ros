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

#include "speak_ros/speak_ros.hpp"

#include <chrono>
#include <functional>

namespace speak_ros
{

SpeakROS::SpeakROS(const rclcpp::NodeOptions & options)
: Node("speak_ros", options),
  class_loader_("speak_ros", "speak_ros::SpeakROSPluginBase"),
  audio_queue_(64),
  audio_player_()
{
  declare_parameter<std::string>("plugin_name", "open_jtalk_plugin::OpenJTalkPlugin");
  get_parameter("plugin_name", plugin_name_);

  // load plugin
  try {
    plugin_ = class_loader_.createSharedInstance(plugin_name_);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Can't find plugin : " << plugin_name_ << " : " << ex.what());
    return;
  }

  plugin_->setParameterInterface(get_node_parameters_interface());
  RCLCPP_INFO_STREAM(get_logger(), "Using plugin : " << plugin_name_);
  plugin_->initializeParameters();

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<Speak>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "speak",
    std::bind(&SpeakROS::handleGoal, this, _1, _2),
    std::bind(&SpeakROS::handleCancel, this, _1),
    std::bind(&SpeakROS::handleAccepted, this, _1));

  parameter_schema_service_ = create_service<GetParameterSchema>(
    "get_parameter_schema",
    std::bind(&SpeakROS::handleGetParameterSchema, this, _1, _2));

  RCLCPP_INFO(get_logger(), "SpeakROS initialized");
}

SpeakROS::~SpeakROS()
{
  if (execution_thread_.joinable()) {
    if (current_cancel_token_) {
      current_cancel_token_->store(true);
    }
    execution_thread_.join();
  }
}

rclcpp_action::GoalResponse SpeakROS::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const Speak::Goal> goal)
{
  if (goal->text.empty()) {
    RCLCPP_WARN(get_logger(), "Rejected empty text goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Accepting goal: '%s'", goal->text.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SpeakROS::handleCancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SpeakROS::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  // cancel previous goal
  {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    if (current_cancel_token_) {
      RCLCPP_INFO(get_logger(), "Canceling previous goal for barge-in");
      current_cancel_token_->store(true);
    }
  }

  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }

  execution_thread_ = std::thread([this, goal_handle]() {
    executeGoal(goal_handle);
  });
}

void SpeakROS::executeGoal(std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Speak::Feedback>();
  auto result = std::make_shared<Speak::Result>();

  CancelToken cancel_token;
  {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    current_cancel_token_ = std::make_shared<std::atomic<bool>>(false);
    cancel_token = current_cancel_token_;
  }

  const auto start_time = now();

  try {
    std::vector<std::pair<std::string, std::string>> overrides;
    for (const auto & param : goal->parameters) {
      overrides.emplace_back(param.name, param.value);
    }
    plugin_->loadParametersWithOverrides(overrides);

    AudioFormat format = plugin_->getAudioFormat();

    audio_queue_.flush();
    audio_player_.open(format);

    // Feedback: SYNTHESIZING
    feedback->state = Speak::Feedback::SYNTHESIZING;
    feedback->synthesis_progress = 0.0;
    feedback->playback_seconds = 0.0;
    feedback->buffer_level = 0.0;
    goal_handle->publish_feedback(feedback);

    std::atomic<uint64_t> played_samples{0};

    std::thread playback_thread([this, &cancel_token, &played_samples]() {
      try {
        audio_player_.play(audio_queue_, cancel_token, played_samples);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Playback error: %s", e.what());
      }
    });

    bool synthesis_success = false;
    try {
      plugin_->synthesize(
        goal->text,
        [this, &cancel_token](AudioChunk chunk) -> bool {
          return audio_queue_.push(chunk, cancel_token);
        },
        cancel_token);
      synthesis_success = !cancel_token->load();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Synthesis error: %s", e.what());
    }

    audio_queue_.markDone();

    while (!audio_player_.isDone() && !cancel_token->load()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(get_logger(), "Goal canceled by client");
        cancel_token->store(true);
        audio_queue_.flush();
        audio_player_.stop();
        break;
      }

      feedback->state = Speak::Feedback::PLAYING;
      feedback->playback_seconds =
        static_cast<float>(played_samples.load()) / format.sample_rate;
      feedback->buffer_level =
        static_cast<float>(audio_queue_.size()) / 64.0f;
      goal_handle->publish_feedback(feedback);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    playback_thread.join();

    result->elapsed_time = (now() - start_time);
    result->total_samples_played = played_samples.load();

    if (cancel_token->load()) {
      result->termination_reason = Speak::Result::CANCELLED;
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Goal canceled");
    } else if (!synthesis_success) {
      result->termination_reason = Speak::Result::ERROR;
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "Goal aborted due to synthesis error");
    } else {
      result->termination_reason = Speak::Result::COMPLETED;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded");
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Execution error: %s", e.what());
    result->elapsed_time = (now() - start_time);
    result->termination_reason = Speak::Result::ERROR;
    goal_handle->abort(result);
  }

  audio_player_.close();

  {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    if (current_cancel_token_ == cancel_token) {
      current_cancel_token_.reset();
    }
  }
}

void SpeakROS::handleGetParameterSchema(
  const std::shared_ptr<GetParameterSchema::Request> /*request*/,
  std::shared_ptr<GetParameterSchema::Response> response)
{
  if (!plugin_) {
    RCLCPP_WARN(get_logger(), "Plugin not loaded, cannot provide parameter schema");
    return;
  }

  auto params_def = plugin_->getParametersDefault();

  for (const auto & param : params_def) {
    speak_ros_interfaces::msg::ParameterSchema schema;
    schema.name = param.name;
    schema.description = param.description;

    if (std::holds_alternative<int>(param.default_value)) {
      schema.type = "int";
      schema.default_value = std::to_string(std::get<int>(param.default_value));
    } else if (std::holds_alternative<double>(param.default_value)) {
      schema.type = "double";
      schema.default_value = std::to_string(std::get<double>(param.default_value));
    } else if (std::holds_alternative<std::string>(param.default_value)) {
      schema.type = "string";
      schema.default_value = std::get<std::string>(param.default_value);
    }

    response->parameters.push_back(schema);
  }

  RCLCPP_INFO(
    get_logger(), "Provided parameter schema with %zu parameters",
    response->parameters.size());
}

}  // namespace speak_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(speak_ros::SpeakROS)
