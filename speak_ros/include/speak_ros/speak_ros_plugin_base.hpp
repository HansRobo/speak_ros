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

#include <filesystem>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace speak_ros
{
struct Parameter
{
  std::string name;
  std::string description;
  std::string default_value;
};

class SpeakROSPluginBase
{
public:
  [[nodiscard]] virtual std::string getPluginName() const = 0;
  virtual std::filesystem::path generateSoundFile(
    const std::string input_text, const std::filesystem::path output_directory,
    const std::string file_name) = 0;
  virtual std::vector<Parameter> getParametersDefault() const { return std::vector<Parameter>(); }
  virtual void importParameters(const std::unordered_map<std::string, std::string> & parameters) {}
  virtual void playSoundFile(std::filesystem::path generated_sound_path)
  {
    std::string command = "aplay " + generated_sound_path.string();
    system(command.c_str());
  }
};
}  // namespace speak_ros

#endif  // SPEAK_ROS__SPEAK_ROS_PLUGIN_HPP_
