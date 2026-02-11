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

#include "speak_ros/wav_utils.hpp"
#include "speak_ros_aivis_plugin/aivis_plugin.hpp"

speak_ros::AudioFormat aivis_plugin::AivisPlugin::getAudioFormat() const
{
  speak_ros::AudioFormat format;
  format.sample_rate = output_sampling_rate;
  format.channels = (output_stereo == "true") ? 2 : 1;
  format.bits_per_sample = 16;
  return format;
}

void aivis_plugin::AivisPlugin::synthesize(
  const std::string & text, speak_ros::AudioChunkCallback callback,
  speak_ros::CancelToken cancel_token)
{
  std::string base_url = "http://" + host_name + ":" + std::to_string(port);

  // Step 1: HTTP POST /audio_query
  auto audio_query_task =
    pplx::create_task([=] {
      web::http::client::http_client_config config;
      web::http::client::http_client client(base_url, config);
      web::json::value body;

      body[U("accent_phrases")] = web::json::value::array();
      body[U("speedScale")] = web::json::value::number(speed_scale);
      body[U("pitchScale")] = web::json::value::number(pitch_scale);
      body[U("intonationScale")] = web::json::value::number(intonation_scale);
      body[U("volumeScale")] = web::json::value::number(volume_scale);
      body[U("tempoDynamicsScale")] = web::json::value::number(tempo_dynamics_scale);
      body[U("prePhonemeLength")] = web::json::value::number(pre_phoneme_length);
      body[U("postPhonemeLength")] = web::json::value::number(post_phoneme_length);
      body[U("outputSamplingRate")] = web::json::value::number(output_sampling_rate);
      body[U("outputStereo")] = web::json::value::string(output_stereo);

      return client.request(
        web::http::methods::POST,
        web::http::uri_builder(U("/audio_query"))
          .append_query(U("text"), text)
          .append_query(U("speaker"), speaker)
          .to_string(),
        web::json::value().serialize(), U("application/json"));
    }).then([cancel_token](web::http::http_response response) {
      if (cancel_token && cancel_token->load()) {
        throw std::runtime_error("cancelled during audio_query");
      }
      if (response.status_code() == web::http::status_codes::OK) {
        return response;
      }
      throw std::runtime_error("audio_query failed");
    });

  audio_query_task.wait();
  auto audio_query_response = audio_query_task.get();

  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Step 2: HTTP POST /synthesis to retrieve entire WAV
  std::vector<uint8_t> wav_data;
  auto synthesis_task =
    pplx::create_task([=]() {
      web::json::value body = audio_query_response.extract_json().get();
      body[U("speedScale")] = web::json::value::number(speed_scale);
      body[U("pitchScale")] = web::json::value::number(pitch_scale);
      body[U("intonationScale")] = web::json::value::number(intonation_scale);
      body[U("volumeScale")] = web::json::value::number(volume_scale);
      body[U("tempoDynamicsScale")] = web::json::value::number(tempo_dynamics_scale);
      body[U("prePhonemeLength")] = web::json::value::number(pre_phoneme_length);
      body[U("postPhonemeLength")] = web::json::value::number(post_phoneme_length);
      body[U("outputSamplingRate")] = web::json::value::number(output_sampling_rate);
      body[U("outputStereo")] = web::json::value::string(output_stereo);

      web::http::client::http_client_config config;
      web::http::client::http_client client(base_url, config);
      return client.request(
        web::http::methods::POST,
        web::http::uri_builder(U("/synthesis")).append_query(U("speaker"), speaker).to_string(),
        body.serialize(), "application/json");
    }).then([cancel_token](web::http::http_response response) {
      if (cancel_token && cancel_token->load()) {
        throw std::runtime_error("cancelled during synthesis");
      }
      if (response.status_code() == web::http::status_codes::OK) {
        return response.extract_vector();
      }
      throw std::runtime_error("synthesis failed");
    });

  wav_data = synthesis_task.get();

  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Step 3: Parse WAV header
  auto header = speak_ros::wav_utils::parseWavHeader(wav_data);

  // Step 4: Extract PCM data
  auto pcm_data = speak_ros::wav_utils::extractPcmData(wav_data);

  // Step 5: Split PCM data into 4096-byte chunks and send
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

std::vector<speak_ros::Parameter> aivis_plugin::AivisPlugin::getParametersDefault() const
{
  return {
    // clang-format off
      {"speaker", "[number/integer] aivis speaker id", 888753760},
      {"host_name", "[string] host of aivis engine", "localhost"},
      {"port", "[number/integer] port of aivis engine", 10101},
      {"speedScale", "[number] voice speed", 1.0},
      {"pitchScale", "[number] voice pitch", 0.0},
      {"intonationScale", "[number] emotional expression (0.0-2.0)", 1.0},
      {"volumeScale", "[number] volume scale", 1.0},
      {"tempoDynamicsScale", "[number] speech speed fluctuation (0.0-2.0)", 1.0},
      {"prePhonemeLength", "[number] pre phoneme length [sec]", 0.1},
      {"postPhonemeLength", "[number] post phoneme length [sec]", 0.1},
      {"outputSamplingRate", "[number] output sampling rate [Hz]", 44100},
      {"outputStereo", "[bool] output stereo", "false"}  // clang-format on
  };
}

void aivis_plugin::AivisPlugin::importParameters(
  const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
{
  speaker = std::get<int>(parameters.at("speaker"));
  host_name = std::get<std::string>(parameters.at("host_name"));
  port = std::get<int>(parameters.at("port"));
  speed_scale = std::get<double>(parameters.at("speedScale"));
  pitch_scale = std::get<double>(parameters.at("pitchScale"));
  intonation_scale = std::get<double>(parameters.at("intonationScale"));
  volume_scale = std::get<double>(parameters.at("volumeScale"));
  tempo_dynamics_scale = std::get<double>(parameters.at("tempoDynamicsScale"));
  pre_phoneme_length = std::get<double>(parameters.at("prePhonemeLength"));
  post_phoneme_length = std::get<double>(parameters.at("postPhonemeLength"));
  output_sampling_rate = std::get<int>(parameters.at("outputSamplingRate"));
  output_stereo = std::get<std::string>(parameters.at("outputStereo"));
}
