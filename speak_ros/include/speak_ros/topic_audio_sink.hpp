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

#ifndef SPEAK_ROS__TOPIC_AUDIO_SINK_HPP_
#define SPEAK_ROS__TOPIC_AUDIO_SINK_HPP_

#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "speak_ros/audio_sink.hpp"
#include "speak_ros_interfaces/msg/audio_chunk.hpp"

namespace speak_ros
{

/**
 * @brief Publishes synthesized PCM as AudioChunk on the "audio_out" topic
 *
 * For robots whose speaker is not reachable through PortAudio. Playback is done by the
 * subscriber, so the goal completes when all chunks are published, not when they finish playing.
 */
class TopicAudioSink : public AudioSink
{
public:
  explicit TopicAudioSink(rclcpp::Node & node);

  void open(const AudioFormat & format) override;
  void play(
    AudioQueue & queue, CancelToken cancel_token, std::atomic<uint64_t> & played_samples) override;
  void stop() override;
  void close() override;
  bool isDone() const override;

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<speak_ros_interfaces::msg::AudioChunk>::SharedPtr publisher_;
  AudioFormat format_;
  std::atomic<bool> is_done_{false};
};

}  // namespace speak_ros

#endif  // SPEAK_ROS__TOPIC_AUDIO_SINK_HPP_
