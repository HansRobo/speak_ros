// Copyright 2026 HansRobo All rights reserved.
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

#ifndef SPEAK_ROS_GEMINI_TTS_PLUGIN__GEMINI_TTS_PLUGIN_HPP_
#define SPEAK_ROS_GEMINI_TTS_PLUGIN__GEMINI_TTS_PLUGIN_HPP_

#include <speak_ros/speak_ros_plugin_base.hpp>

namespace gemini_tts_plugin
{
class GeminiTTSPlugin : public speak_ros::SpeakROSPluginBase
{
public:
  std::string getPluginName() const override { return "gemini_tts_plugin"; }
  speak_ros::AudioFormat getAudioFormat() const override;
  void synthesize(
    const std::string & text, speak_ros::AudioChunkCallback callback,
    speak_ros::CancelToken cancel_token) override;
  std::vector<speak_ros::Parameter> getParametersDefault() const override;
  void importParameters(
    const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
    override;

private:
  std::string base_url_ = "https://generativelanguage.googleapis.com";
  std::string model_ = "gemini-3.8-flash-tts";
  std::string voice_ = "Kore";
  std::string style_;
  std::string api_key_;
};
}  // namespace gemini_tts_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gemini_tts_plugin::GeminiTTSPlugin, speak_ros::SpeakROSPluginBase)
#endif
