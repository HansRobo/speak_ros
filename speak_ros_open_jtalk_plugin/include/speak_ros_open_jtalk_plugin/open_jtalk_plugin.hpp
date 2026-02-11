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

#ifndef OPEN_JTALK_PLUGIN__OPEN_JTALK_PLUGIN_HPP_
#define OPEN_JTALK_PLUGIN__OPEN_JTALK_PLUGIN_HPP_

#include <speak_ros/speak_ros_plugin_base.hpp>

namespace open_jtalk_plugin
{
class OpenJTalkPlugin : public speak_ros::SpeakROSPluginBase
{
public:
  OpenJTalkPlugin() : speak_ros::SpeakROSPluginBase() {}

  std::string getPluginName() const override { return "open_jtalk_plugin"; }

  speak_ros::AudioFormat getAudioFormat() const override;

  void synthesize(
    const std::string & text, speak_ros::AudioChunkCallback callback,
    speak_ros::CancelToken cancel_token) override;

  std::vector<speak_ros::Parameter> getParametersDefault() const override;

  void importParameters(
    const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
    override;

  std::string dictionary_path;
  std::string hts_voice_path;
  std::string speed_rate;
};
}  // namespace open_jtalk_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(open_jtalk_plugin::OpenJTalkPlugin, speak_ros::SpeakROSPluginBase)

#endif  // OPEN_JTALK_PLUGIN__OPEN_JTALK_PLUGIN_HPP_
