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

#ifndef SPEAK_ROS_AIVIS_PLUGIN__AIVIS_PLUGIN_HPP_
#define SPEAK_ROS_AIVIS_PLUGIN__AIVIS_PLUGIN_HPP_

#include <speak_ros/speak_ros_plugin_base.hpp>

namespace aivis_plugin
{
class AivisPlugin : public speak_ros::SpeakROSPluginBase
{
public:
  AivisPlugin() : speak_ros::SpeakROSPluginBase() {}

  std::string getPluginName() const override { return "aivis_plugin"; }

  speak_ros::AudioFormat getAudioFormat() const override;

  void synthesize(
    const std::string & text, speak_ros::AudioChunkCallback callback,
    std::shared_ptr<std::atomic<bool>> cancel_token) override;

  std::vector<speak_ros::Parameter> getParametersDefault() const override;

  void importParameters(
    const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
    override;

  int speaker;
  std::string host_name;
  int port;
  double speed_scale;
  double pitch_scale;
  double intonation_scale;
  double volume_scale;
  double tempo_dynamics_scale;
  double pre_phoneme_length;
  double post_phoneme_length;
  int output_sampling_rate;
  std::string output_stereo;
};
}  // namespace aivis_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aivis_plugin::AivisPlugin, speak_ros::SpeakROSPluginBase)

#endif  // SPEAK_ROS_AIVIS_PLUGIN__AIVIS_PLUGIN_HPP_
