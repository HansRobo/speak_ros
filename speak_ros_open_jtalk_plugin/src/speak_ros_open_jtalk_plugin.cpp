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

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "speak_ros/wav_utils.hpp"
#include "speak_ros_open_jtalk_plugin/open_jtalk_plugin.hpp"

speak_ros::AudioFormat open_jtalk_plugin::OpenJTalkPlugin::getAudioFormat() const
{
  // OpenJTalk default output format
  speak_ros::AudioFormat format;
  format.sample_rate = 48000;   // OpenJTalk default is 48kHz
  format.channels = 1;          // Monaural
  format.bits_per_sample = 16;  // 16bit
  return format;
}

void open_jtalk_plugin::OpenJTalkPlugin::synthesize(
  const std::string & text, speak_ros::AudioChunkCallback callback,
  speak_ros::CancelToken cancel_token)
{
  // Generate temporary file path
  std::string temp_file_path = std::tmpnam(nullptr);
  temp_file_path += ".wav";

  // Build OpenJTalk command
  std::stringstream command_ss;
  // clang-format off
  command_ss << "echo \"" << text << "\""
             << " | open_jtalk"
             << " -x "  << dictionary_path
             << " -m "  << hts_voice_path
             << " -ow " << temp_file_path
             << " -r "  << speed_rate;
  // clang-format on

  // Check for cancellation
  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Execute OpenJTalk to generate WAV file
  int result = system(command_ss.str().c_str());
  if (result != 0) {
    throw std::runtime_error("OpenJTalk execution failed");
  }

  // Check for cancellation
  if (cancel_token && cancel_token->load()) {
    std::filesystem::remove(temp_file_path);
    return;
  }

  // Load generated WAV file
  std::ifstream file(temp_file_path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Failed to open generated WAV file");
  }

  std::vector<uint8_t> wav_data(
    (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  // Delete temporary file
  std::filesystem::remove(temp_file_path);

  // Check for cancellation
  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Parse WAV header
  speak_ros::wav_utils::WavHeader header;
  try {
    header = speak_ros::wav_utils::parseWavHeader(wav_data);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to parse WAV header: ") + e.what());
  }

  // Extract PCM data
  const uint8_t * pcm_data = wav_data.data() + header.data_offset;
  const size_t pcm_size = header.data_size;

  // Chunk size (approximately every 100ms)
  const size_t bytes_per_frame = header.channels * (header.bits_per_sample / 8);
  const size_t frames_per_chunk = header.sample_rate / 10;  // 100ms
  const size_t chunk_size = frames_per_chunk * bytes_per_frame;

  // Split PCM data into chunks and send via callback
  size_t offset = 0;
  while (offset < pcm_size) {
    // Check for cancellation
    if (cancel_token && cancel_token->load()) {
      return;
    }

    size_t remaining = pcm_size - offset;
    size_t current_chunk_size = std::min(chunk_size, remaining);

    speak_ros::AudioChunk chunk;
    chunk.data.assign(pcm_data + offset, pcm_data + offset + current_chunk_size);
    chunk.is_final = (offset + current_chunk_size >= pcm_size);

    // Invoke callback (abort if false is returned)
    if (!callback(std::move(chunk))) {
      return;
    }

    offset += current_chunk_size;
  }
}

std::vector<speak_ros::Parameter> open_jtalk_plugin::OpenJTalkPlugin::getParametersDefault() const
{
  return {
    // clang-format off
      {"dictionary_path", "[string] dictionary path", "/var/lib/mecab/dic/open-jtalk/naist-jdic"},
      {"hts_voice_path", "[string] hts voice file path", "/usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice"},
      {"speed_rate", "[string] speed rate", "1.0"}
    // clang-format on
  };
}

void open_jtalk_plugin::OpenJTalkPlugin::importParameters(
  const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
{
  dictionary_path = std::get<std::string>(parameters.at("dictionary_path"));
  hts_voice_path = std::get<std::string>(parameters.at("hts_voice_path"));
  speed_rate = std::get<std::string>(parameters.at("speed_rate"));
}
