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

#include "speak_ros/http_stream.hpp"

#include <chrono>

namespace speak_ros
{
web::http::client::http_client_config streamingConfig()
{
  web::http::client::http_client_config config;
  config.set_chunksize(READ_SIZE);
  return config;
}

HttpStream::HttpStream(
  const std::string & base_url, speak_ros::CancelToken cancel_token,
  const web::http::client::http_client_config & config)
: client_(base_url, config), cancel_token_(std::move(cancel_token)), watcher_([this] {
    while (!finished_.load()) {
      if (isCancelled()) {
        cancel();
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  })
{
}

HttpStream::~HttpStream()
{
  // Decoder/status exceptions must not leave the response producer running.
  try {
    cancel();
  } catch (...) {
    // Keep destruction non-throwing and always join the watcher.
  }
  finished_.store(true);
  watcher_.join();
}

bool HttpStream::isCancelled() const { return cancel_token_ && cancel_token_->load(); }

void HttpStream::cancel() { cancellation_.cancel(); }

web::http::http_response HttpStream::request(web::http::http_request request)
{
  if (isCancelled()) {
    cancel();
  }
  return client_.request(std::move(request), cancellation_.get_token()).get();
}

bool HttpStream::consume(
  web::http::http_response response, const std::function<bool(const uint8_t *, size_t)> & consumer)
{
  auto input = response.body();
  uint8_t bytes[READ_SIZE];
  while (!isCancelled()) {
    const auto count = input.streambuf().getn(bytes, sizeof(bytes)).get();
    if (isCancelled()) {
      break;
    }
    if (count == 0) {
      return true;
    }
    if (!consumer(bytes, count)) {
      break;
    }
  }
  cancel();
  return false;
}
}  // namespace speak_ros
