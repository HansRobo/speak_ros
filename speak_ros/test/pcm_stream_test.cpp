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

#include "speak_ros/pcm_stream.hpp"

#include <iostream>

int main()
{
  bool ok = true;
  for (const auto channels : {1, 2}) {
    speak_ros::AudioFormat format;
    format.channels = channels;
    std::vector<uint8_t> source(20000);
    for (size_t i = 0; i < source.size(); ++i) {
      source[i] = static_cast<uint8_t>(i);
    }
    for (const size_t split : {1, 3, 4095, 4096, 20000}) {
      std::vector<uint8_t> actual;
      int finals = 0;
      speak_ros::PcmStream pcm(format, [&](speak_ros::AudioChunk chunk) {
        ok = ok && chunk.data.size() <= 4096 && chunk.data.size() % format.bytesPerFrame() == 0 &&
             finals == 0;
        finals += chunk.is_final;
        actual.insert(actual.end(), chunk.data.begin(), chunk.data.end());
        return true;
      });
      for (size_t offset = 0; offset < source.size(); offset += split) {
        ok = pcm.feed(source.data() + offset, std::min(split, source.size() - offset)) && ok;
      }
      pcm.finish();
      ok = ok && actual == source && finals == 1;
    }
  }
  speak_ros::AudioFormat format;
  int calls = 0;
  speak_ros::PcmStream empty(format, [&](speak_ros::AudioChunk chunk) {
    ok = ok && chunk.is_final && chunk.data.empty();
    ++calls;
    return true;
  });
  empty.finish();
  ok = ok && calls == 1;
  const uint8_t bytes[] = {1, 2, 3};
  speak_ros::PcmStream odd(format, [&](speak_ros::AudioChunk) { return false; });
  ok = odd.feed(bytes, 1) && ok;
  try {
    odd.finish();
    ok = false;
  } catch (const std::runtime_error &) {
  }
  speak_ros::PcmStream rejected(format, [&](speak_ros::AudioChunk chunk) {
    ok = ok && !chunk.is_final;
    return false;
  });
  ok = !rejected.feed(bytes, 2) && ok;
  format.channels = 0;
  try {
    speak_ros::PcmStream invalid(format, [](speak_ros::AudioChunk) { return true; });
    ok = false;
  } catch (const std::invalid_argument &) {
  }
  if (!ok) {
    std::cerr << "PCM stream contract failed\n";
  }
  return ok ? 0 : 1;
}
