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

#include "speak_ros/audio_queue.hpp"

namespace speak_ros
{

AudioQueue::AudioQueue(size_t max_chunks) : max_chunks_(max_chunks) {}

bool AudioQueue::push(const AudioChunk & chunk, CancelToken cancel_token)
{
  std::unique_lock<std::mutex> lock(mutex_);

  // Wait until there's space in the queue (or cancel/flush)
  cv_not_full_.wait(lock, [this, &cancel_token]() {
    return queue_.size() < max_chunks_ || (cancel_token && cancel_token->load()) || flushed_;
  });

  // If cancelled or flushed
  if ((cancel_token && cancel_token->load()) || flushed_) {
    return false;
  }

  queue_.push(chunk);
  cv_not_empty_.notify_one();
  return true;
}

std::optional<AudioChunk> AudioQueue::pop(CancelToken cancel_token)
{
  std::unique_lock<std::mutex> lock(mutex_);

  // Wait until data arrives (or cancel/done/flush)
  cv_not_empty_.wait(lock, [this, &cancel_token]() {
    return !queue_.empty() || done_ || (cancel_token && cancel_token->load()) || flushed_;
  });

  // If cancelled or flushed
  if ((cancel_token && cancel_token->load()) || flushed_) {
    return std::nullopt;
  }

  // If queue is empty and producer is done
  if (queue_.empty() && done_) {
    return std::nullopt;
  }

  // If data exists
  if (!queue_.empty()) {
    AudioChunk chunk = queue_.front();
    queue_.pop();
    cv_not_full_.notify_one();
    return chunk;
  }

  return std::nullopt;
}

void AudioQueue::flush()
{
  std::unique_lock<std::mutex> lock(mutex_);

  // Clear the queue
  while (!queue_.empty()) {
    queue_.pop();
  }

  // Temporarily set flush flag to true to release waiting operations
  flushed_ = true;

  // Release waiting push/pop operations
  cv_not_full_.notify_all();
  cv_not_empty_.notify_all();

  // Make reusable after flush
  flushed_ = false;
  done_ = false;
}

void AudioQueue::markDone()
{
  std::unique_lock<std::mutex> lock(mutex_);
  done_ = true;
  flushed_ = false;  // Reset flush flag when done
  cv_not_empty_.notify_all();
}

size_t AudioQueue::size() const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return queue_.size();
}

bool AudioQueue::isDone() const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return done_;
}

}  // namespace speak_ros
