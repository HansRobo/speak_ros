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

#ifndef WAV_STREAM_DECODER_HPP_
#define WAV_STREAM_DECODER_HPP_

#include <algorithm>
#include <cstring>
#include <speak_ros/audio_types.hpp>
#include <stdexcept>

namespace voicevox_plugin
{
// Incremental RIFF decoder. Transport boundaries never become PCM boundaries.
class WavStreamDecoder
{
public:
  WavStreamDecoder(speak_ros::AudioFormat format, speak_ros::AudioChunkCallback callback)
  : format_(format), callback_(std::move(callback))
  {
  }

  bool feed(const uint8_t * bytes, size_t size)
  {
    received_ += size;
    if (riff_ && received_ > expected_) {
      fail("RIFF size exceeded");
    }
    buffer_.insert(buffer_.end(), bytes, bytes + size);
    if (!riff_) {
      if (buffer_.size() < 12) {
        return true;
      }
      if (!id(0, "RIFF") || !id(8, "WAVE")) {
        fail("Invalid RIFF/WAVE header");
      }
      expected_ = uint64_t(u32(4)) + 8;
      if (expected_ < 44 || received_ > expected_) {
        fail("Invalid RIFF size");
      }
      discard(12);
      riff_ = true;
    }
    while (!data_) {
      if (buffer_.size() < 8) {
        return true;
      }
      const size_t length = u32(4);
      if (id(0, "data")) {
        if (!fmt_ || length % format_.bytesPerFrame()) {
          fail("Invalid PCM data size");
        }
        remaining_ = length;
        discard(8);
        data_ = true;
        break;
      }
      // Headers have a strict bound; never buffer arbitrary metadata sizes.
      if (length > 65536) {
        fail("WAV metadata exceeds limit");
      }
      const size_t padded = length + (length & 1);
      if (buffer_.size() < 8 + padded) {
        return true;
      }
      if (id(0, "fmt ")) {
        if (
          length < 16 || u16(8) != 1 || u16(10) != format_.channels ||
          u32(12) != format_.sample_rate || u16(22) != format_.bits_per_sample ||
          u16(20) != format_.bytesPerFrame()) {
          fail("Unexpected WAV format");
        }
        fmt_ = true;
      }
      discard(8 + padded);
    }
    if (buffer_.size() > remaining_) {
      fail("Unexpected bytes after PCM data");
    }
    size_t count = std::min(buffer_.size(), size_t(4096));
    count -= count % format_.bytesPerFrame();
    while (count) {
      speak_ros::AudioChunk chunk;
      chunk.data.assign(buffer_.begin(), buffer_.begin() + count);
      discard(count);
      remaining_ -= count;
      if (!callback_(std::move(chunk))) {
        return false;
      }
      count = std::min(buffer_.size(), size_t(4096));
      count -= count % format_.bytesPerFrame();
    }
    return true;
  }

  void finish()
  {
    if (!data_ || remaining_ || !buffer_.empty() || received_ != expected_) {
      fail("Truncated WAV stream");
    }
    speak_ros::AudioChunk end;
    end.is_final = true;
    callback_(std::move(end));
  }

private:
  [[noreturn]] static void fail(const char * message) { throw std::runtime_error(message); }
  bool id(size_t offset, const char * value) const
  {
    return std::memcmp(buffer_.data() + offset, value, 4) == 0;
  }
  uint16_t u16(size_t offset) const
  {
    return buffer_[offset] | (uint16_t(buffer_[offset + 1]) << 8);
  }
  uint32_t u32(size_t offset) const { return u16(offset) | (uint32_t(u16(offset + 2)) << 16); }
  void discard(size_t size) { buffer_.erase(buffer_.begin(), buffer_.begin() + size); }
  speak_ros::AudioFormat format_;
  speak_ros::AudioChunkCallback callback_;
  std::vector<uint8_t> buffer_;
  uint64_t received_ = 0;
  uint64_t expected_ = 0;
  size_t remaining_ = 0;
  bool riff_ = false;
  bool fmt_ = false;
  bool data_ = false;
};
}  // namespace voicevox_plugin
#endif
