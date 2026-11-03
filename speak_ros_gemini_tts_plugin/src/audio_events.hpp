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

#ifndef SPEAK_ROS_GEMINI_TTS_PLUGIN__AUDIO_EVENTS_HPP_
#define SPEAK_ROS_GEMINI_TTS_PLUGIN__AUDIO_EVENTS_HPP_

#include <cpprest/json.h>

#include <cctype>
#include <speak_ros/pcm_stream.hpp>
#include <sstream>

namespace gemini_tts_plugin
{
class AudioEvents
{
public:
  explicit AudioEvents(speak_ros::PcmStream & pcm) : pcm_(pcm) {}

  bool feed(const uint8_t * bytes, size_t size)
  {
    for (size_t i = 0; i < size; ++i) {
      if (++event_bytes_ > MAX_EVENT_BYTES) {
        fail("Gemini SSE event exceeds size limit");
      }
      if (bytes[i] == '\n') {
        if (!line_.empty() && line_.back() == '\r') {
          line_.pop_back();
        }
        if (!line()) {
          return false;
        }
        line_.clear();
      } else {
        line_.push_back(static_cast<char>(bytes[i]));
      }
    }
    return true;
  }

  void finish() const
  {
    if (!line_.empty() || !data_.empty() || !event_name_.empty() || !completed_) {
      fail("Gemini stream ended without a complete successful interaction");
    }
  }

private:
  [[noreturn]] static void fail(const char * message) { throw std::runtime_error(message); }

  bool line()
  {
    if (line_.empty()) {
      const bool accepted = data_.empty() || dispatch();
      data_.clear();
      event_name_.clear();
      event_bytes_ = 0;
      return accepted;
    }
    if (line_[0] == ':') {
      return true;
    }
    const auto colon = line_.find(':');
    const auto field = line_.substr(0, colon);
    const auto start =
      colon == std::string::npos
        ? line_.size()
        : colon + 1 + (colon + 1 < line_.size() && line_[colon + 1] == ' ' ? 1 : 0);
    if (field == "data") {
      data_.append(line_, start, std::string::npos);
      data_.push_back('\n');
    } else if (field == "event") {
      event_name_ = line_.substr(start);
    }
    return true;
  }

  static int digit(char c)
  {
    if (c >= 'A' && c <= 'Z') {
      return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
      return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
      return c - '0' + 52;
    }
    if (c == '+') {
      return 62;
    }
    if (c == '/') {
      return 63;
    }
    return -1;
  }

  static std::vector<uint8_t> decode(const std::string & text)
  {
    if (text.size() % 4) {
      fail("Invalid Gemini audio base64 length");
    }
    std::vector<uint8_t> bytes;
    bytes.reserve(text.size() / 4 * 3);
    for (size_t i = 0; i < text.size(); i += 4) {
      const int a = digit(text[i]);
      const int b = digit(text[i + 1]);
      const int c = digit(text[i + 2]);
      const int d = digit(text[i + 3]);
      const bool pad_c = text[i + 2] == '=';
      const bool pad_d = text[i + 3] == '=';
      if (
        a < 0 || b < 0 || (!pad_c && c < 0) || (!pad_d && d < 0) || (pad_c && !pad_d) ||
        ((pad_c || pad_d) && i + 4 != text.size()) || (pad_c && (b & 15)) ||
        (pad_d && !pad_c && (c & 3))) {
        fail("Invalid Gemini audio base64 encoding");
      }
      bytes.push_back(static_cast<uint8_t>((a << 2) | (b >> 4)));
      if (!pad_c) {
        bytes.push_back(static_cast<uint8_t>((b << 4) | (c >> 2)));
      }
      if (!pad_d) {
        bytes.push_back(static_cast<uint8_t>((c << 6) | d));
      }
    }
    return bytes;
  }

  static void validateFormat(const web::json::value & audio)
  {
    if (
      (audio.has_field(U("sample_rate")) && audio.at(U("sample_rate")).as_double() != 24000.0) ||
      (audio.has_field(U("channels")) && audio.at(U("channels")).as_double() != 1.0)) {
      fail("Unexpected Gemini audio rate or channels");
    }
    if (audio.has_field(U("mime_type"))) {
      auto mime = audio.at(U("mime_type")).as_string();
      for (auto & c : mime) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      }
      std::istringstream parts(mime);
      std::string part;
      std::getline(parts, part, ';');
      if (part != "audio/l16") {
        fail("Unexpected Gemini audio encoding");
      }
      while (std::getline(parts, part, ';')) {
        part.erase(0, part.find_first_not_of(" \t"));
        part.erase(part.find_last_not_of(" \t") + 1);
        if (part != "rate=24000" && part != "channels=1" && part != "codec=pcm") {
          fail("Unexpected Gemini PCM parameters");
        }
      }
    }
  }

  bool audio(const web::json::value & value)
  {
    validateFormat(value);
    if (!value.has_field(U("data"))) {
      if (value.has_field(U("uri"))) {
        fail("Gemini URI audio is unsupported; inline PCM is required");
      }
      return true;
    }
    const auto bytes = decode(value.at(U("data")).as_string());
    return pcm_.feed(bytes.data(), bytes.size());
  }

  bool dispatch()
  {
    if (completed_) {
      fail("Unexpected Gemini event after completion");
    }
    data_.pop_back();
    const auto event = web::json::value::parse(data_);
    const auto type = event.at(U("event_type")).as_string();
    if (!event_name_.empty() && event_name_ != type) {
      fail("Gemini SSE event type mismatch");
    }
    if (
      event.has_field(U("error")) || type == "error" || type == "interaction.failed" ||
      type == "interaction.cancelled") {
      fail("Gemini interaction failed");
    }
    if (event.has_field(U("status"))) {
      const auto status = event.at(U("status")).as_string();
      if (status != "in_progress" && status != "queued" && status != "completed") {
        fail("Gemini interaction did not complete successfully");
      }
    }
    if (event.has_field(U("interaction")) && event.at(U("interaction")).has_field(U("status"))) {
      const auto status = event.at(U("interaction")).at(U("status")).as_string();
      if (status != "in_progress" && status != "queued" && status != "completed") {
        fail("Gemini interaction did not complete successfully");
      }
    }
    if (type == "interaction.completed") {
      if (event.at(U("interaction")).at(U("status")).as_string() != "completed") {
        fail("Gemini interaction did not complete successfully");
      }
      completed_ = true;
    } else if (type == "step.delta") {
      const auto & delta = event.at(U("delta"));
      if (delta.at(U("type")).as_string() == "audio") {
        return audio(delta);
      }
    } else if (type == "step.start") {
      const auto & step = event.at(U("step"));
      if (step.has_field(U("error"))) {
        fail("Gemini model output failed");
      }
      if (step.at(U("type")).as_string() == "model_output" && step.has_field(U("content"))) {
        for (const auto & content : step.at(U("content")).as_array()) {
          if (content.at(U("type")).as_string() == "audio" && !audio(content)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  static constexpr size_t MAX_EVENT_BYTES = 4 * 1024 * 1024;
  speak_ros::PcmStream & pcm_;
  std::string line_;
  std::string data_;
  std::string event_name_;
  size_t event_bytes_ = 0;
  bool completed_ = false;
};
}  // namespace gemini_tts_plugin
#endif
