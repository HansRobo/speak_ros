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


#include "speak_ros/topic_audio_sink.hpp"

#include <optional>
#include <stdexcept>

namespace speak_ros
{

TopicAudioSink::TopicAudioSink(rclcpp::Node & node) : node_(node)
{
  publisher_ = node_.create_publisher<speak_ros_interfaces::msg::AudioChunk>(
    "audio_out", rclcpp::QoS(100).reliable());
}

void TopicAudioSink::open(const AudioFormat & format)
{
  format_ = format;
  is_done_.store(false);
}

void TopicAudioSink::play(
  AudioQueue & queue, CancelToken cancel_token, std::atomic<uint64_t> & played_samples)
{
  played_samples.store(0);

  while (true) {
    if (cancel_token && cancel_token->load()) {
      break;
    }
    std::optional<AudioChunk> chunk = queue.pop(cancel_token);
    if (!chunk.has_value()) {
      break;
    }

    if (!chunk->data.empty()) {
      speak_ros_interfaces::msg::AudioChunk msg;
      msg.timestamp = node_.now();
      msg.data = chunk->data;
      msg.sample_rate = format_.sample_rate;
      msg.channels = static_cast<uint8_t>(format_.channels);
      msg.encoding = "pcm_s" + std::to_string(format_.bits_per_sample) + "le";
      publisher_->publish(msg);
      played_samples.fetch_add(chunk->data.size() / format_.bytesPerFrame());
    }

    if (chunk->is_final) {
      break;
    }
  }

  is_done_.store(true);
}

void TopicAudioSink::stop() { is_done_.store(true); }

void TopicAudioSink::close() {}

bool TopicAudioSink::isDone() const { return is_done_.load(); }

}  // namespace speak_ros
