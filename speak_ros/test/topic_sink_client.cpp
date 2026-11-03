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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "speak_ros/topic_audio_sink.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("topic_sink_contract_test");
  speak_ros::TopicAudioSink sink(*node);
  using Message = speak_ros_interfaces::msg::AudioChunk;
  std::vector<Message> received;
  auto subscription = node->create_subscription<Message>(
    "audio_out", rclcpp::QoS(100).reliable(),
    [&](Message::ConstSharedPtr message) { received.push_back(*message); });
  auto spin_until = [&](auto predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
    return predicate();
  };
  bool ok = spin_until([&] { return subscription->get_publisher_count() == 1; });
  auto token = std::make_shared<std::atomic<bool>>(false);
  std::atomic<uint64_t> played{0};
  speak_ros::AudioFormat format;
  format.sample_rate = 16000;
  sink.open(format);
  speak_ros::AudioQueue queue(4);
  std::thread consumer([&] { sink.play(queue, token, played); });
  // Dummy producer intentionally remains unfinished until PCM arrives over ROS.
  speak_ros::AudioChunk first;
  first.data = {0x34, 0x12};
  queue.push(first, token);
  ok = spin_until([&] { return received.size() == 1; }) && ok;
  speak_ros::AudioChunk end;
  end.is_final = true;
  queue.push(end, token);
  consumer.join();
  ok = spin_until([&] { return received.size() == 2; }) && ok;
  ok = ok && played.load() == 1 && sink.isDone();
  if (received.size() == 2) {
    ok = ok && !received[0].stream_id.empty() && received[0].stream_id == received[1].stream_id &&
         received[0].data == first.data && !received[0].is_final && received[1].data.empty() &&
         received[1].is_final && received[0].sample_rate == 16000 && received[0].channels == 1 &&
         received[0].encoding == "pcm_s16le";
  }
  // A one-shot producer uses the same interface and receives a new stream ID.
  speak_ros::AudioQueue second_queue(4);
  first.is_final = true;
  second_queue.push(first, token);
  sink.open(format);
  sink.play(second_queue, token, played);
  ok = spin_until([&] { return received.size() == 3; }) && ok;
  if (received.size() == 3) {
    ok = ok && received[2].is_final && received[2].data == first.data &&
         received[2].stream_id != received[0].stream_id;
  }
  // A cancelled invocation must not publish queued PCM or a successful end.
  speak_ros::AudioQueue cancelled_queue(4);
  cancelled_queue.push(first, token);
  token->store(true);
  sink.open(format);
  sink.play(cancelled_queue, token, played);
  rclcpp::spin_some(node);
  ok = ok && received.size() == 3 && played.load() == 0;
  rclcpp::shutdown();
  if (!ok) {
    std::cerr << "TopicAudioSink contract failed\n";
  }
  return ok ? 0 : 1;
}
