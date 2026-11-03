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

#include <cpprest/http_client.h>

#include <cctype>
#include <cstdlib>
#include <speak_ros_http/http_stream.hpp>
#include <speak_ros_http/pcm_stream.hpp>

#include "audio_events.hpp"
#include "speak_ros_gemini_tts_plugin/gemini_tts_plugin.hpp"

namespace gemini_tts_plugin
{
speak_ros::AudioFormat GeminiTTSPlugin::getAudioFormat() const
{
  speak_ros::AudioFormat format;
  format.sample_rate = 24000;
  format.channels = 1;
  format.bits_per_sample = 16;
  return format;
}

void GeminiTTSPlugin::synthesize(
  const std::string & text, speak_ros::AudioChunkCallback callback,
  speak_ros::CancelToken cancel_token)
{
  if (cancel_token && cancel_token->load()) {
    return;
  }
  std::string key = api_key_;
  if (key.empty()) {
    const char * env = std::getenv("GEMINI_API_KEY");
    if (env) {
      key = env;
    }
  }
  if (key.empty()) {
    throw std::runtime_error("Set GEMINI_API_KEY or the api_key parameter for Gemini TTS");
  }
  auto config = speak_ros_http::streamingConfig();
  config.set_validate_certificates(true);
  config.set_max_redirects(0);
  speak_ros_http::HttpStream http(base_url_, cancel_token, config);

  using web::json::value;
  value content;
  content[U("type")] = value::string(U("text"));
  content[U("text")] = value::string(text);
  if (!style_.empty()) {
    value annotation;
    annotation[U("type")] = value::string(U("speech_metadata"));
    annotation[U("style")] = value::string(style_);
    content[U("annotations")] = value::array({annotation});
  }
  value input;
  input[U("type")] = value::string(U("user_input"));
  input[U("content")] = value::array({content});
  value voice;
  voice[U("voice")] = value::string(voice_);
  value body;
  body[U("model")] = value::string(model_);
  body[U("input")] = value::array({input});
  body[U("response_format")][U("type")] = value::string(U("audio"));
  body[U("response_format")][U("mime_type")] = value::string(U("audio/l16"));
  body[U("response_format")][U("sample_rate")] = value::number(24000);
  body[U("generation_config")][U("speech_config")] = value::array({voice});
  body[U("stream")] = value::boolean(true);
  body[U("store")] = value::boolean(false);

  web::http::http_request request(web::http::methods::POST);
  request.set_request_uri(U("/v1beta/interactions"));
  request.headers().add(U("x-goog-api-key"), key);
  request.headers().add(U("Accept"), U("text/event-stream"));
  request.set_body(body);
  auto response = http.request(std::move(request));
  if (response.status_code() != web::http::status_codes::OK) {
    throw std::runtime_error("Gemini TTS HTTP status " + std::to_string(response.status_code()));
  }
  auto content_type = response.headers().content_type();
  content_type = content_type.substr(0, content_type.find(';'));
  content_type.erase(0, content_type.find_first_not_of(" \t"));
  content_type.erase(content_type.find_last_not_of(" \t") + 1);
  for (auto & c : content_type) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (content_type != U("text/event-stream")) {
    throw std::runtime_error("Gemini TTS response is not an event stream");
  }
  speak_ros_http::PcmStream pcm(getAudioFormat(), std::move(callback));
  AudioEvents events(pcm);
  if (http.consume(
        response, [&](const uint8_t * bytes, size_t size) { return events.feed(bytes, size); })) {
    events.finish();
    pcm.finish();
  }
}

std::vector<speak_ros::Parameter> GeminiTTSPlugin::getParametersDefault() const
{
  return {
    {"base_url", "[string] trusted API origin (receives the API key)",
     "https://generativelanguage.googleapis.com"},
    {"model", "[string] Gemini Interactions TTS model", "gemini-3.8-flash-tts"},
    {"voice", "[string] Gemini voice", "Kore"},
    {"style", "[string] optional speech style", ""},
    {"api_key", "[string] API key (empty uses GEMINI_API_KEY)", ""}};
}

void GeminiTTSPlugin::importParameters(
  const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
{
  base_url_ = std::get<std::string>(parameters.at("base_url"));
  const web::uri origin(base_url_);
  if (
    (origin.scheme() != U("https") && origin.scheme() != U("http")) || origin.host().empty() ||
    !origin.user_info().empty() || (!origin.path().empty() && origin.path() != U("/")) ||
    !origin.query().empty() || !origin.fragment().empty()) {
    throw std::invalid_argument("base_url must be an HTTP(S) origin without credentials or path");
  }
  model_ = std::get<std::string>(parameters.at("model"));
  voice_ = std::get<std::string>(parameters.at("voice"));
  style_ = std::get<std::string>(parameters.at("style"));
  api_key_ = std::get<std::string>(parameters.at("api_key"));
}
}  // namespace gemini_tts_plugin
