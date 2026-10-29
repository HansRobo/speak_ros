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

#ifndef SPEAK_ROS__AUDIO_SINK_HPP_
#define SPEAK_ROS__AUDIO_SINK_HPP_

#include <atomic>

#include "speak_ros/audio_queue.hpp"
#include "speak_ros/audio_types.hpp"

namespace speak_ros
{

/**
 * @brief Audio output destination (speaker, ROS topic, ...)
 *
 * SpeakROS opens the sink, runs play() in a thread to drain AudioQueue,
 * and stop()/close() it when finished or cancelled.
 */
class AudioSink
{
public:
  virtual ~AudioSink() = default;

  virtual void open(const AudioFormat & format) = 0;
  virtual void play(
    AudioQueue & queue, CancelToken cancel_token, std::atomic<uint64_t> & played_samples) = 0;
  virtual void stop() = 0;
  virtual void close() = 0;
  virtual bool isDone() const = 0;
};

}  // namespace speak_ros

#endif  // SPEAK_ROS__AUDIO_SINK_HPP_
