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

#include <chrono>
#include <iostream>
#include <thread>

#include "speak_ros_openai_tts_plugin/openai_tts_plugin.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {
    return 2;
  }
  openai_tts_plugin::OpenAITTSPlugin plugin;
  std::unordered_map<std::string, std::variant<int, double, std::string>> parameters;
  for (const auto & p : plugin.getParametersDefault()) {
    parameters[p.name] = p.default_value;
  }
  parameters["base_url"] = std::string("http://127.0.0.1:") + argv[1];
  parameters["api_key"] = std::string("mock-key");
  parameters["instructions"] = std::string("calm");
  if (std::string(argv[2]) == "invalid_url") {
    for (const auto * url :
         {"ftp://example.com", "https://example.com/v1", "https://user:password@example.com",
          "https://example.com?q=1", "https://example.com/#fragment"}) {
      parameters["base_url"] = std::string(url);
      try {
        plugin.importParameters(parameters);
        return 9;
      } catch (const std::exception &) {
      }
    }
    return 0;
  }
  plugin.importParameters(parameters);
  const std::string mode(argv[2]);
  auto token = std::make_shared<std::atomic<bool>>(false);
  std::thread canceller;
  if (mode == "cancel") {
    canceller = std::thread([token] {
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      token->store(true);
    });
  }
  const auto start = std::chrono::steady_clock::now();
  auto seconds = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  };
  bool threw = false;
  bool valid = true;
  size_t bytes = 0;
  int calls = 0;
  int finals = 0;
  double first = -1;
  try {
    plugin.synthesize(
      "streaming test",
      [&](speak_ros::AudioChunk chunk) {
        ++calls;
        if (!chunk.data.empty() && first < 0) {
          first = seconds();
        }
        valid = valid && chunk.data.size() % 2 == 0;
        for (size_t i = 0; i < chunk.data.size(); ++i) {
          valid = valid && chunk.data[i] == ((i % 2 == 0) ? 0x34 : 0x12);
        }
        if (finals != 0) {
          valid = false;
        }
        bytes += chunk.data.size();
        finals += chunk.is_final ? 1 : 0;
        return mode != "abort";
      },
      token);
  } catch (const std::exception & e) {
    threw = true;
    std::cerr << e.what() << '\n';
  }
  const double elapsed = seconds();
  if (canceller.joinable()) {
    canceller.join();
  }
  std::cout << "bytes=" << bytes << " finals=" << finals << " first=" << first
            << " elapsed=" << elapsed << " calls=" << calls << " threw=" << threw << '\n';
  if (!valid) {
    return 3;
  }
  if (mode == "error") {
    return threw && finals == 0 ? 0 : 4;
  }
  if (mode == "cancel") {
    return token->load() && elapsed < 2.0 && finals == 0 ? 0 : 5;
  }
  if (mode == "abort") {
    return !threw && calls == 1 && finals == 0 && elapsed < 1.0 ? 0 : 6;
  }
  if (threw || bytes != (mode == "empty" ? 0u : 12000u) || finals != 1) {
    return 7;
  }
  if (mode == "early" && !(first >= 0 && first < 1.0)) {
    return 8;
  }
  return 0;
}
