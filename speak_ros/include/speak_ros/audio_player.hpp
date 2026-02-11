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

#ifndef SPEAK_ROS__AUDIO_PLAYER_HPP_
#define SPEAK_ROS__AUDIO_PLAYER_HPP_

#include <atomic>
#include <chrono>
#include <memory>

#include "speak_ros/audio_queue.hpp"
#include "speak_ros/audio_types.hpp"

// PortAudio forward declaration
typedef void PaStream;

namespace speak_ros
{

/**
 * @brief Audio playback using PortAudio
 *
 * Uses blocking I/O mode (Pa_WriteStream) to immediately play
 * chunks obtained from AudioQueue.
 */
class AudioPlayer
{
public:
  /**
   * @brief Constructor
   *
   * Calls Pa_Initialize().
   */
  AudioPlayer();

  /**
   * @brief Destructor
   *
   * Calls Pa_Terminate().
   */
  ~AudioPlayer();

  // Disable copy/move
  AudioPlayer(const AudioPlayer &) = delete;
  AudioPlayer & operator=(const AudioPlayer &) = delete;
  AudioPlayer(AudioPlayer &&) = delete;
  AudioPlayer & operator=(AudioPlayer &&) = delete;

  /**
   * @brief Open stream
   *
   * @param format Audio format
   * @throw std::runtime_error On stream open failure
   */
  void open(const AudioFormat & format);

  /**
   * @brief Blocking playback loop
   *
   * Retrieves chunks from AudioQueue and plays them with Pa_WriteStream().
   * If cancel_token becomes true, stops immediately with Pa_AbortStream().
   * On normal completion, stops with Pa_StopStream() after playing remaining buffer.
   *
   * @param queue Audio chunk queue
   * @param cancel_token Cancel token
   * @param played_samples Number of played samples (for feedback)
   */
  void play(AudioQueue & queue, CancelToken cancel_token, std::atomic<uint64_t> & played_samples);

  /**
   * @brief Stop playback immediately
   *
   * Calls Pa_AbortStream() (discards buffer).
   */
  void stop();

  /**
   * @brief Close stream
   *
   * Calls Pa_CloseStream().
   */
  void close();

  /**
   * @brief Wait for playback completion
   *
   * @param timeout Timeout duration
   * @return True if completed before timeout
   */
  bool waitForDone(std::chrono::milliseconds timeout);

  /**
   * @brief Check if playback is complete
   *
   * @return True if completed
   */
  bool isDone() const;

private:
  PaStream * stream_ = nullptr;
  AudioFormat format_;
  std::atomic<bool> is_done_{false};
  bool initialized_ = false;
};

}  // namespace speak_ros

#endif  // SPEAK_ROS__AUDIO_PLAYER_HPP_
