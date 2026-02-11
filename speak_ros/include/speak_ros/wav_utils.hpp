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

#ifndef SPEAK_ROS__WAV_UTILS_HPP_
#define SPEAK_ROS__WAV_UTILS_HPP_

#include "speak_ros/audio_types.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace speak_ros
{
namespace wav_utils
{

/**
 * @brief WAV header information
 */
struct WavHeader
{
  uint32_t sample_rate;       // Sampling rate
  uint16_t channels;          // Number of channels
  uint16_t bits_per_sample;   // Bits per sample
  uint32_t data_offset;       // Start offset of PCM data
  uint32_t data_size;         // Size of PCM data (bytes)
};

/**
 * @brief Parse WAV header
 *
 * @param wav_data Byte sequence of the entire WAV file
 * @return WAV header information
 * @throw std::runtime_error On parse failure
 */
WavHeader parseWavHeader(const std::vector<uint8_t> & wav_data);

/**
 * @brief Extract PCM data from WAV file
 *
 * @param wav_data Byte sequence of the entire WAV file
 * @return PCM data only (without header)
 * @throw std::runtime_error On parse failure
 */
std::vector<uint8_t> extractPcmData(const std::vector<uint8_t> & wav_data);

/**
 * @brief Convert WavHeader to AudioFormat
 *
 * @param header WAV header
 * @return AudioFormat
 */
AudioFormat toAudioFormat(const WavHeader & header);

}  // namespace wav_utils
}  // namespace speak_ros

#endif  // SPEAK_ROS__WAV_UTILS_HPP_
