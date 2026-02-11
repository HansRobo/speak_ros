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

#ifndef SPEAK_ROS_VOICEVOX_PLUGIN__VOICEVOX_PLUGIN_HPP
#define SPEAK_ROS_VOICEVOX_PLUGIN__VOICEVOX_PLUGIN_HPP

#include <speak_ros/speak_ros_plugin_base.hpp>

namespace voicevox_plugin
{
class VoiceVoxPlugin : public speak_ros::SpeakROSPluginBase
{
public:
  VoiceVoxPlugin() : speak_ros::SpeakROSPluginBase() {}

  std::string getPluginName() const override { return "voicevox_plugin"; }

  speak_ros::AudioFormat getAudioFormat() const override;

  void synthesize(
    const std::string & text, speak_ros::AudioChunkCallback callback,
    speak_ros::CancelToken cancel_token) override;

  std::vector<speak_ros::Parameter> getParametersDefault() const override;

  void importParameters(
    const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
    override;

  int speaker;
  std::string host_name;
  int port;
  double speed_scale;
  double pitch_scale;
  double intensity_scale;
  double volume_scale;
  double pre_phoneme_length;
  double post_phoneme_length;
  int output_sampling_rate;
  std::string output_stereo;
};
}  // namespace voicevox_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voicevox_plugin::VoiceVoxPlugin, speak_ros::SpeakROSPluginBase)

#endif  // SPEAK_ROS_VOICEVOX_PLUGIN__VOICEVOX_PLUGIN_HPP
