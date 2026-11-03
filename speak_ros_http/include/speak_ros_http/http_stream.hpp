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

#ifndef SPEAK_ROS_HTTP__HTTP_STREAM_HPP_
#define SPEAK_ROS_HTTP__HTTP_STREAM_HPP_

#include <cpprest/http_client.h>

#include <speak_ros/audio_types.hpp>
#include <thread>

namespace speak_ros_http
{
constexpr size_t READ_SIZE = 4096;

web::http::client::http_client_config streamingConfig();

// Owns one synthesis operation's HTTP cancellation and incremental body reads.
// Request schemas, credentials, status handling and decoding belong to callers.
class HttpStream
{
public:
  HttpStream(
    const std::string & base_url, speak_ros::CancelToken cancel_token,
    const web::http::client::http_client_config & config = streamingConfig());
  ~HttpStream();
  HttpStream(const HttpStream &) = delete;
  HttpStream & operator=(const HttpStream &) = delete;

  web::http::http_response request(web::http::http_request request);
  // True means normal EOF. False means cancellation or consumer rejection.
  bool consume(
    web::http::http_response response,
    const std::function<bool(const uint8_t *, size_t)> & consumer);
  void cancel();

private:
  bool isCancelled() const;
  web::http::client::http_client client_;
  speak_ros::CancelToken cancel_token_;
  pplx::cancellation_token_source cancellation_;
  std::atomic<bool> finished_{false};
  std::thread watcher_;
};
}  // namespace speak_ros_http
#endif
