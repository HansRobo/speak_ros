// Copyright 2026 HansRobo All rights reserved.
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

#ifndef SPEAK_ROS_HTTP__PCM_STREAM_HPP_
#define SPEAK_ROS_HTTP__PCM_STREAM_HPP_

#include <algorithm>
#include <speak_ros/audio_types.hpp>
#include <stdexcept>

namespace speak_ros_http
{
// Aligns arbitrary transport fragments without accumulating an utterance.
class PcmStream
{
public:
  PcmStream(speak_ros::AudioFormat format, speak_ros::AudioChunkCallback callback)
  : frame_size_(format.bytesPerFrame()), callback_(std::move(callback))
  {
    if (frame_size_ == 0 || frame_size_ > 4096) {
      throw std::invalid_argument("Invalid PCM frame size");
    }
  }

  bool feed(const uint8_t * bytes, size_t size)
  {
    while (size) {
      const size_t count = std::min(size, size_t(4096) - pending_.size());
      pending_.insert(pending_.end(), bytes, bytes + count);
      bytes += count;
      size -= count;
      const size_t aligned = pending_.size() - pending_.size() % frame_size_;
      if (aligned) {
        speak_ros::AudioChunk chunk;
        chunk.data.assign(pending_.begin(), pending_.begin() + aligned);
        pending_.erase(pending_.begin(), pending_.begin() + aligned);
        if (!callback_(std::move(chunk))) {
          return false;
        }
      }
    }
    return true;
  }

  void finish()
  {
    if (!pending_.empty()) {
      throw std::runtime_error("Incomplete PCM frame at end of response");
    }
    speak_ros::AudioChunk end;
    end.is_final = true;
    callback_(std::move(end));
  }

private:
  size_t frame_size_;
  speak_ros::AudioChunkCallback callback_;
  std::vector<uint8_t> pending_;
};
}  // namespace speak_ros_http
#endif
