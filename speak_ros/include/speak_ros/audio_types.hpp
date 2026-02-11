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

#ifndef SPEAK_ROS__AUDIO_TYPES_HPP_
#define SPEAK_ROS__AUDIO_TYPES_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace speak_ros
{

/**
 * @brief Audio format definition
 */
struct AudioFormat
{
  uint32_t sample_rate = 24000;     // Sampling rate (Hz)
  uint16_t channels = 1;             // Number of channels (1=mono, 2=stereo)
  uint16_t bits_per_sample = 16;    // Bits per sample

  /**
   * @brief Calculate bytes per frame
   * @return Number of bytes
   */
  uint32_t bytesPerFrame() const
  {
    return channels * (bits_per_sample / 8);
  }
};

/**
 * @brief Audio data chunk
 */
struct AudioChunk
{
  std::vector<uint8_t> data;  // Raw PCM data (no WAV header)
  bool is_final = false;       // Final chunk marker
};

/**
 * @brief Cancel token
 *
 * Set to true to request cancellation of running operations.
 */
using CancelToken = std::shared_ptr<std::atomic<bool>>;

/**
 * @brief Audio chunk callback
 *
 * @param chunk Audio chunk
 * @return true=continue, false=abort request (e.g., queue full)
 */
using AudioChunkCallback = std::function<bool(AudioChunk)>;

}  // namespace speak_ros

#endif  // SPEAK_ROS__AUDIO_TYPES_HPP_
