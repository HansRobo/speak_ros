// Copyright 2022 Kotaro Yoshimoto All rights reserved.
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

#include <cmath>

#include "speak_ros/http_stream.hpp"
#include "speak_ros_voicevox_plugin/voicevox_plugin.hpp"
#include "wav_stream_decoder.hpp"

speak_ros::AudioFormat voicevox_plugin::VoiceVoxPlugin::getAudioFormat() const
{
  speak_ros::AudioFormat format;
  format.sample_rate = output_sampling_rate;
  format.channels = (output_stereo == "true") ? 2 : 1;
  format.bits_per_sample = 16;
  return format;
}

void voicevox_plugin::VoiceVoxPlugin::synthesize(
  const std::string & text, speak_ros::AudioChunkCallback callback,
  speak_ros::CancelToken cancel_token)
{
  const std::string base_url = "http://" + host_name + ":" + std::to_string(port);
  speak_ros::HttpStream http(base_url, cancel_token);
  auto request = [&](
                   const web::http::method & method, const utility::string_t & uri,
                   const utility::string_t & body = utility::string_t()) {
    web::http::http_request req(method);
    req.set_request_uri(uri);
    if (!body.empty()) {
      req.set_body(body, U("application/json"));
    }
    return http.request(std::move(req));
  };
  bool streaming = false;
  auto manifest = request(web::http::methods::GET, U("/engine_manifest"));
  if (manifest.status_code() != web::http::status_codes::NotFound) {
    if (manifest.status_code() != web::http::status_codes::OK) {
      throw std::runtime_error("engine_manifest failed");
    }
    auto value = manifest.extract_json().get();
    if (!value.is_object()) {
      throw std::runtime_error("Invalid engine manifest");
    }
    if (value.has_field(U("supported_features"))) {
      auto features = value.at(U("supported_features"));
      if (!features.is_object()) {
        throw std::runtime_error("Invalid supported_features");
      }
      if (features.has_field(U("streaming_synthesis"))) {
        streaming = features.at(U("streaming_synthesis")).at(U("value")).as_bool();
      }
    }
  }
  auto query_uri = web::http::uri_builder(U("/audio_query"))
                     .append_query(U("text"), text)
                     .append_query(U("speaker"), speaker)
                     .to_string();
  auto query = request(web::http::methods::POST, query_uri);
  if (query.status_code() != web::http::status_codes::OK) {
    throw std::runtime_error("audio_query failed");
  }
  auto body = query.extract_json().get();
  body[U("speedScale")] = web::json::value::number(speed_scale);
  body[U("pitchScale")] = web::json::value::number(pitch_scale);
  body[U("volumeScale")] = web::json::value::number(volume_scale);
  body[U("prePhonemeLength")] = web::json::value::number(pre_phoneme_length);
  body[U("postPhonemeLength")] = web::json::value::number(post_phoneme_length);
  body[U("outputSamplingRate")] = web::json::value::number(output_sampling_rate);
  body[U("outputStereo")] = web::json::value::boolean(output_stereo == "true");
  web::http::uri_builder uri(streaming ? U("/streaming_synthesis") : U("/synthesis"));
  uri.append_query(U("speaker"), speaker);
  if (streaming) {
    uri.append_query(U("segment_length"), segment_length);
  }
  auto response = request(web::http::methods::POST, uri.to_string(), body.serialize());
  if (response.status_code() != web::http::status_codes::OK) {
    throw std::runtime_error("synthesis failed");
  }
  WavStreamDecoder decoder(getAudioFormat(), std::move(callback));
  if (http.consume(
        response, [&](const uint8_t * bytes, size_t size) { return decoder.feed(bytes, size); })) {
    decoder.finish();
  }
}

std::vector<speak_ros::Parameter> voicevox_plugin::VoiceVoxPlugin::getParametersDefault() const
{
  return {
    // clang-format off
      {"segment_length", "[number] streaming segment length [sec]", 0.3},
      {"speaker", "[number/integer] voicevox speaker id", 1},
      {"host_name", "[string] host of voicevox engine", "localhost"},
      {"port", "[number/integer] port of voicevox engine", 50021},
      {"speedScale", "[number] voice speed. larger is faster", 2.0},
      {"pitchScale", "[number] voice pitch. larger is higher", 0.0},
      {"intensityScale", "[number] intensity scale. larger is more intense", 1.0},
      {"volumeScale", "[number] volume scale. larger is more volume", 1.0},
      {"prePhonemeLength", "[number] pre phoneme length [sec]", 0.1},
      {"postPhonemeLength", "[number] post phoneme length [sec]", 0.1},
      {"outputSamplingRate", "[number] output sampling rate [Hz]", 24000},
      {"outputStereo", "[bool] output stereo", "true"}  // clang-format on
  };
}

void voicevox_plugin::VoiceVoxPlugin::importParameters(
  const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
{
  segment_length = std::get<double>(parameters.at("segment_length"));
  if (!std::isfinite(segment_length) || segment_length <= 0) {
    throw std::invalid_argument("segment_length must be positive and finite");
  }
  speaker = std::get<int>(parameters.at("speaker"));
  host_name = std::get<std::string>(parameters.at("host_name"));
  port = std::get<int>(parameters.at("port"));
  speed_scale = std::get<double>(parameters.at("speedScale"));
  pitch_scale = std::get<double>(parameters.at("pitchScale"));
  intensity_scale = std::get<double>(parameters.at("intensityScale"));
  volume_scale = std::get<double>(parameters.at("volumeScale"));
  pre_phoneme_length = std::get<double>(parameters.at("prePhonemeLength"));
  post_phoneme_length = std::get<double>(parameters.at("postPhonemeLength"));
  output_sampling_rate = std::get<int>(parameters.at("outputSamplingRate"));
  output_stereo = std::get<std::string>(parameters.at("outputStereo"));
}
