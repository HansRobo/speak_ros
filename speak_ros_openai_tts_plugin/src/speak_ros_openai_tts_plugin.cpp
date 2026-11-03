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

#include <cpprest/http_client.h>

#include <chrono>
#include <cstdlib>
#include <thread>

#include "speak_ros_openai_tts_plugin/openai_tts_plugin.hpp"

speak_ros::AudioFormat openai_tts_plugin::OpenAITTSPlugin::getAudioFormat() const
{
  speak_ros::AudioFormat format;
  // OpenAI TTS API pcm format is fixed at 24kHz/16-bit/mono
  format.sample_rate = 24000;
  format.channels = 1;
  format.bits_per_sample = 16;
  return format;
}

void openai_tts_plugin::OpenAITTSPlugin::synthesize(
  const std::string & text, speak_ros::AudioChunkCallback callback,
  speak_ros::CancelToken cancel_token)
{
  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Get API key (parameter takes priority, fall back to environment variable)
  std::string effective_api_key = api_key;
  if (effective_api_key.empty()) {
    const char * env_key = std::getenv("OPENAI_API_KEY");
    if (env_key) {
      effective_api_key = env_key;
    }
  }

  if (effective_api_key.empty()) {
    throw std::runtime_error(
      "OpenAI API key not found. Set OPENAI_API_KEY environment variable or 'api_key' parameter.");
  }

  // Prepare HTTP POST request
  web::http::client::http_client_config config;
  // Enable HTTPS certificate validation (required for production)
  config.set_validate_certificates(true);
  config.set_chunksize(4096);
  config.set_max_redirects(0);

  web::http::client::http_client client(base_url, config);

  // Build request body
  web::json::value body;
  body[U("model")] = web::json::value::string(model);
  body[U("input")] = web::json::value::string(text);
  body[U("voice")] = web::json::value::string(voice);
  body[U("response_format")] = web::json::value::string("pcm");
  body[U("speed")] = web::json::value::number(speed);

  // instructions is an optional parameter for gpt-4o-mini-tts (exclude if empty)
  if (!instructions.empty()) {
    body[U("instructions")] = web::json::value::string(instructions);
  }

  // Send HTTP request
  web::http::http_request request(web::http::methods::POST);
  request.set_request_uri(U("/v1/audio/speech"));
  request.headers().add(U("Authorization"), U("Bearer ") + effective_api_key);
  request.headers().set_content_type(U("application/json"));
  request.set_body(body);

  pplx::cancellation_token_source cancellation;
  std::atomic<bool> finished{false};
  std::thread watcher([&] {
    while (!finished.load()) {
      if (cancel_token && cancel_token->load()) {
        cancellation.cancel();
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  struct Join
  {
    std::atomic<bool> & finished;
    std::thread & thread;
    ~Join()
    {
      finished.store(true);
      thread.join();
    }
  } join{finished, watcher};
  auto response_task = client.request(request, cancellation.get_token());

  // Check for cancellation
  if (cancel_token && cancel_token->load()) {
    cancellation.cancel();
    return;
  }

  web::http::http_response response;
  try {
    response = response_task.get();
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("OpenAI TTS API request failed: ") + e.what());
  }

  // Check status code
  if (response.status_code() != web::http::status_codes::OK) {
    std::string error_msg =
      "OpenAI TTS API returned status code: " + std::to_string(response.status_code());

    throw std::runtime_error(error_msg);
  }

  auto input = response.body();
  std::vector<uint8_t> pending;
  uint8_t bytes[4096];
  while (!(cancel_token && cancel_token->load())) {
    const auto count = input.streambuf().getn(bytes, sizeof(bytes)).get();
    if (cancel_token && cancel_token->load()) {
      cancellation.cancel();
      return;
    }
    if (count == 0) {
      if (!pending.empty()) {
        throw std::runtime_error("Incomplete PCM frame at end of OpenAI TTS response");
      }
      speak_ros::AudioChunk end;
      end.is_final = true;
      callback(std::move(end));
      return;
    }
    pending.insert(pending.end(), bytes, bytes + count);
    const size_t aligned = pending.size() - pending.size() % getAudioFormat().bytesPerFrame();
    if (aligned) {
      speak_ros::AudioChunk chunk;
      chunk.data.assign(pending.begin(), pending.begin() + aligned);
      pending.erase(pending.begin(), pending.begin() + aligned);
      if (!callback(std::move(chunk))) {
        cancellation.cancel();
        return;
      }
    }
  }
  cancellation.cancel();
}

std::vector<speak_ros::Parameter> openai_tts_plugin::OpenAITTSPlugin::getParametersDefault() const
{
  return {
    // clang-format off
      {"base_url", "[string] trusted API endpoint (receives the API key)", "https://api.openai.com"},
      {"model", "[string] TTS model (tts-1, tts-1-hd, gpt-4o-mini-tts)", "tts-1"},
      {"voice", "[string] Voice name (alloy, echo, fable, onyx, nova, shimmer)", "nova"},
      {"speed", "[number] Playback speed (0.25 - 4.0)", 1.0},
      {"instructions", "[string] Style instructions for gpt-4o-mini-tts (empty to disable)", ""},
      {"api_key", "[string] OpenAI API key (empty to use OPENAI_API_KEY env var)", ""}
    // clang-format on
  };
}

void openai_tts_plugin::OpenAITTSPlugin::importParameters(
  const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
{
  base_url = std::get<std::string>(parameters.at("base_url"));
  const web::uri endpoint(base_url);
  if (
    (endpoint.scheme() != U("http") && endpoint.scheme() != U("https")) ||
    endpoint.host().empty() || !endpoint.user_info().empty() ||
    (!endpoint.path().empty() && endpoint.path() != U("/")) || !endpoint.query().empty() ||
    !endpoint.fragment().empty()) {
    throw std::invalid_argument("base_url must be an HTTP(S) origin without credentials or path");
  }
  model = std::get<std::string>(parameters.at("model"));
  voice = std::get<std::string>(parameters.at("voice"));
  speed = std::get<double>(parameters.at("speed"));
  instructions = std::get<std::string>(parameters.at("instructions"));
  api_key = std::get<std::string>(parameters.at("api_key"));
}
