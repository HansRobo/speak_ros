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

#include "../src/audio_events.hpp"

#include <iostream>

int main()
{
  const std::string events =
    ": comment\r\n"
    "event: step.start\r\n"
    "data: {\"event_type\":\"step.start\",\r\n"
    "data: \"step\":{\"type\":\"model_output\",\"content\":[{\"type\":\"audio\","
    "\"data\":\"NA==\"}]}}\r\n\r\n"
    "data: {\"event_type\":\"step.delta\",\"delta\":{\"type\":\"audio\","
    "\"data\":\"EjQS\"}}\n\n"
    "data: {\"event_type\":\"interaction.completed\","
    "\"interaction\":{\"status\":\"completed\"}}\n\n";
  bool ok = true;
  for (const size_t split : {1, 2, 3, 17, 4096}) {
    int finals = 0;
    std::vector<uint8_t> bytes;
    speak_ros::AudioFormat format;
    speak_ros::PcmStream pcm(format, [&](speak_ros::AudioChunk chunk) {
      ok = ok && chunk.data.size() % 2 == 0;
      bytes.insert(bytes.end(), chunk.data.begin(), chunk.data.end());
      finals += chunk.is_final;
      return true;
    });
    gemini_tts_plugin::AudioEvents parser(pcm);
    for (size_t offset = 0; offset < events.size(); offset += split) {
      ok = parser.feed(
             reinterpret_cast<const uint8_t *>(events.data() + offset),
             std::min(split, events.size() - offset)) &&
           ok;
    }
    parser.finish();
    pcm.finish();
    ok = ok && finals == 1 && bytes == std::vector<uint8_t>({0x34, 0x12, 0x34, 0x12});
  }
  if (!ok) {
    std::cerr << "Gemini audio event fragmentation contract failed\n";
  }
  return ok ? 0 : 1;
}
