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

#ifndef SPEAK_ROS__AUDIO_QUEUE_HPP_
#define SPEAK_ROS__AUDIO_QUEUE_HPP_

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

#include "speak_ros/audio_types.hpp"

namespace speak_ros
{

/**
 * @brief Thread-safe bounded audio chunk queue
 *
 * Buffer between producer (TTS synthesis) and consumer (audio playback).
 * - bounded: push blocks when maximum chunk count is exceeded
 * - flushable: discards queue contents on playback interruption
 * - CancelToken support: releases waiting operations on cancellation
 */
class AudioQueue
{
public:
  /**
   * @brief Constructor
   * @param max_chunks Maximum number of chunks (default 64)
   */
  explicit AudioQueue(size_t max_chunks = 64);

  /**
   * @brief Add chunk to queue
   *
   * Waits until space becomes available if queue is full.
   * Returns false immediately if cancel_token becomes true.
   *
   * @param chunk Audio chunk
   * @param cancel_token Cancel token
   * @return true=success, false=cancelled
   */
  bool push(const AudioChunk & chunk, CancelToken cancel_token);

  /**
   * @brief Remove chunk from queue
   *
   * Waits until data arrives if queue is empty.
   * Returns nullopt if cancel_token is true or markDone() was called.
   *
   * @param cancel_token Cancel token
   * @return Chunk (on success), or nullopt (on finish/cancel)
   */
  std::optional<AudioChunk> pop(CancelToken cancel_token);

  /**
   * @brief Discard all chunks in queue
   *
   * Releases waiting push/pop operations.
   */
  void flush();

  /**
   * @brief Notify producer completion
   *
   * Indicates that no more chunks will be added.
   * Waiting pop() will return nullopt after queue becomes empty.
   */
  void markDone();

  /**
   * @brief Get current queue depth
   * @return Number of chunks
   */
  size_t size() const;

  /**
   * @brief Check if queue is in done state
   * @return True if markDone() has been called
   */
  bool isDone() const;

private:
  const size_t max_chunks_;
  std::queue<AudioChunk> queue_;
  mutable std::mutex mutex_;
  std::condition_variable cv_not_full_;   // Notifies when space becomes available in queue
  std::condition_variable cv_not_empty_;  // Notifies when data enters queue
  bool done_ = false;                     // Producer completion flag
  bool flushed_ = false;                  // Flush request flag
};

}  // namespace speak_ros

#endif  // SPEAK_ROS__AUDIO_QUEUE_HPP_
