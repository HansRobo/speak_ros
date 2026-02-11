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

#include <cstdlib>

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
  std::string base_url = "https://api.openai.com";

  web::http::client::http_client_config config;
  // Enable HTTPS certificate validation (required for production)
  config.set_validate_certificates(true);

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

  // Execute async request synchronously
  auto response_task = client.request(request);

  // Check for cancellation
  if (cancel_token && cancel_token->load()) {
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

    // Include error response body if available
    try {
      auto error_body = response.extract_string().get();
      if (!error_body.empty()) {
        error_msg += ", body: " + error_body;
      }
    } catch (...) {
      // Ignore if error body extraction fails
    }

    throw std::runtime_error(error_msg);
  }

  // Extract entire response body (due to cpprestsdk limitations, must retrieve all at once)
  std::vector<uint8_t> pcm_data;
  try {
    pcm_data = response.extract_vector().get();
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to extract audio data: ") + e.what());
  }

  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Split PCM data into 4096-byte chunks and send
  constexpr size_t CHUNK_SIZE = 4096;
  size_t offset = 0;

  while (offset < pcm_data.size()) {
    if (cancel_token && cancel_token->load()) {
      return;
    }

    size_t remaining = pcm_data.size() - offset;
    size_t chunk_size = std::min(CHUNK_SIZE, remaining);

    speak_ros::AudioChunk chunk;
    chunk.data.assign(pcm_data.begin() + offset, pcm_data.begin() + offset + chunk_size);
    chunk.is_final = (offset + chunk_size >= pcm_data.size());

    // Send chunk via callback
    bool should_continue = callback(std::move(chunk));

    if (!should_continue) {
      return;
    }

    offset += chunk_size;
  }
}

std::vector<speak_ros::Parameter> openai_tts_plugin::OpenAITTSPlugin::getParametersDefault() const
{
  return {
    // clang-format off
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
  model = std::get<std::string>(parameters.at("model"));
  voice = std::get<std::string>(parameters.at("voice"));
  speed = std::get<double>(parameters.at("speed"));
  instructions = std::get<std::string>(parameters.at("instructions"));
  api_key = std::get<std::string>(parameters.at("api_key"));
}
