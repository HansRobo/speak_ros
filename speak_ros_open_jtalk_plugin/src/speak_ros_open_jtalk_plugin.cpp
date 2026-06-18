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

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "speak_ros/wav_utils.hpp"
#include "speak_ros_open_jtalk_plugin/open_jtalk_plugin.hpp"

extern char ** environ;

namespace open_jtalk_plugin
{

namespace
{

// Wall-clock budget for a single open_jtalk invocation. Bounds how long a hung
// child (missing dictionary, defunct process, ...) can stall synthesis. Without
// this the synthesize thread - and, via the executor-side join, the whole node -
// could block forever.
constexpr std::chrono::seconds kSynthesisTimeout{60};

// Removes a temporary file on every control-flow path (return / throw).
struct TempFileGuard
{
  std::string path;
  ~TempFileGuard()
  {
    if (!path.empty()) {
      std::error_code ec;
      std::filesystem::remove(path, ec);
    }
  }
};

// Create a unique temporary file via mkstemp and return its path (fd closed).
std::string makeTempFile(const char * name_template)
{
  const std::string tmpl = (std::filesystem::temp_directory_path() / name_template).string();
  std::vector<char> buffer(tmpl.begin(), tmpl.end());
  buffer.push_back('\0');

  const int fd = mkstemp(buffer.data());
  if (fd == -1) {
    throw std::runtime_error(
      std::string("Failed to create temporary file: ") + std::strerror(errno));
  }
  close(fd);
  return std::string(buffer.data());
}

// Write the synthesis text to a private temporary file used as open_jtalk's stdin.
// Passing the text via a file (instead of a shell pipe) keeps it out of any shell
// and avoids both SIGPIPE handling and write-side deadlocks on a hung child.
std::string writeStdinFile(const std::string & text)
{
  const std::string tmpl =
    (std::filesystem::temp_directory_path() / "speak_ros_open_jtalk_in_XXXXXX").string();
  std::vector<char> buffer(tmpl.begin(), tmpl.end());
  buffer.push_back('\0');

  const int fd = mkstemp(buffer.data());
  if (fd == -1) {
    throw std::runtime_error(
      std::string("Failed to create temporary input file: ") + std::strerror(errno));
  }
  const std::string path(buffer.data());

  // open_jtalk historically received its text from `echo`, which appends a newline.
  const std::string payload = text + "\n";
  const char * p = payload.data();
  size_t remaining = payload.size();
  while (remaining > 0) {
    const ssize_t n = write(fd, p, remaining);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      const std::string err = std::strerror(errno);
      close(fd);
      std::error_code ec;
      std::filesystem::remove(path, ec);
      throw std::runtime_error("Failed to write temporary input file: " + err);
    }
    p += n;
    remaining -= static_cast<size_t>(n);
  }
  close(fd);
  return path;
}

// Validate that `value` is a plain number (full string consumed). open_jtalk's -r
// flag expects a numeric speech rate; rejecting non-numeric input also gives a
// clear error instead of a cryptic open_jtalk failure.
//
// Uses std::from_chars (locale-independent) rather than std::stod: std::stod
// honors LC_NUMERIC, so under a comma-decimal locale it would reject even the
// default "1.0", and it also silently accepts trailing garbage ("1.5x").
void validateNumeric(const std::string & name, const std::string & value)
{
  double parsed = 0.0;
  const char * first = value.data();
  const char * last = value.data() + value.size();
  const auto [ptr, ec] = std::from_chars(first, last, parsed);
  if (ec != std::errc() || ptr != last) {
    throw std::runtime_error("Parameter '" + name + "' must be numeric, got: '" + value + "'");
  }
}

// Reason a still-running child was asked to stop.
// clang-format off
enum class StopReason { kNone, kCancel, kTimeout };  // clang-format on

// Run open_jtalk WITHOUT a shell: arguments are passed as discrete argv entries
// (so nothing in them can be interpreted by a shell) and the text is fed via a
// file redirected onto stdin. The child is polled so a cancel request or the
// wall-clock timeout terminates it (SIGTERM, then SIGKILL) and it is always reaped.
//
// Returns normally on success or on cancellation (caller distinguishes via the
// cancel token); throws std::runtime_error on spawn failure, timeout, or a
// non-zero open_jtalk exit.
void runOpenJTalk(
  const std::vector<std::string> & args, const std::string & stdin_path,
  const speak_ros::CancelToken & cancel_token)
{
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (const auto & arg : args) {
    // posix_spawnp does not modify argv, so const_cast is safe here.
    argv.push_back(const_cast<char *>(arg.c_str()));
  }
  argv.push_back(nullptr);

  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, stdin_path.c_str(), O_RDONLY, 0);

  pid_t pid = -1;
  const int rc = posix_spawnp(&pid, args[0].c_str(), &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);

  if (rc != 0) {
    throw std::runtime_error(std::string("Failed to launch open_jtalk: ") + std::strerror(rc));
  }

  const auto deadline = std::chrono::steady_clock::now() + kSynthesisTimeout;
  StopReason reason = StopReason::kNone;
  std::chrono::steady_clock::time_point term_time;
  bool kill_sent = false;

  while (true) {
    int status = 0;
    const pid_t r = waitpid(pid, &status, WNOHANG);

    if (r == pid) {
      if (reason == StopReason::kCancel || (cancel_token && cancel_token->load())) {
        return;  // cancelled: caller observes the token and treats this as a cancel
      }
      if (reason == StopReason::kTimeout) {
        throw std::runtime_error("open_jtalk timed out");
      }
      if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        return;
      }
      if (WIFSIGNALED(status)) {
        throw std::runtime_error(
          "open_jtalk terminated by signal " + std::to_string(WTERMSIG(status)));
      }
      throw std::runtime_error(
        "open_jtalk execution failed (exit code " +
        std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1) + ")");
    }
    if (r < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(std::string("waitpid failed: ") + std::strerror(errno));
    }

    // Child still running: decide whether to stop it, then escalate to SIGKILL.
    const auto now = std::chrono::steady_clock::now();
    if (reason == StopReason::kNone) {
      if (cancel_token && cancel_token->load()) {
        reason = StopReason::kCancel;
      } else if (now >= deadline) {
        reason = StopReason::kTimeout;
      }
      if (reason != StopReason::kNone) {
        kill(pid, SIGTERM);
        term_time = now;
      }
    } else if (!kill_sent && now - term_time > std::chrono::seconds(2)) {
      kill(pid, SIGKILL);
      kill_sent = true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}  // namespace

}  // namespace open_jtalk_plugin

speak_ros::AudioFormat open_jtalk_plugin::OpenJTalkPlugin::getAudioFormat() const
{
  // OpenJTalk default output format
  speak_ros::AudioFormat format;
  format.sample_rate = 48000;   // OpenJTalk default is 48kHz
  format.channels = 1;          // Monaural
  format.bits_per_sample = 16;  // 16bit
  return format;
}

void open_jtalk_plugin::OpenJTalkPlugin::synthesize(
  const std::string & text, speak_ros::AudioChunkCallback callback,
  speak_ros::CancelToken cancel_token)
{
  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Validate parameters up front. The shell-injection fix below (argv-based exec,
  // no shell) is what actually neutralizes malicious input; these checks add
  // defense-in-depth and clear error messages.
  validateNumeric("speed_rate", speed_rate);
  if (!std::filesystem::exists(dictionary_path)) {
    throw std::runtime_error("dictionary_path does not exist: " + dictionary_path);
  }
  if (!std::filesystem::exists(hts_voice_path)) {
    throw std::runtime_error("hts_voice_path does not exist: " + hts_voice_path);
  }

  // Temporary files are removed on every return/throw path by RAII.
  TempFileGuard stdin_file{writeStdinFile(text)};
  TempFileGuard wav_file{makeTempFile("speak_ros_open_jtalk_XXXXXX")};

  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Run open_jtalk directly (no shell): text via stdin file, all parameters as
  // discrete argv entries so none of them can break out into a command.
  runOpenJTalk(
    {"open_jtalk", "-x", dictionary_path, "-m", hts_voice_path, "-ow", wav_file.path, "-r",
     speed_rate},
    stdin_file.path, cancel_token);

  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Load generated WAV file
  std::ifstream file(wav_file.path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Failed to open generated WAV file");
  }
  std::vector<uint8_t> wav_data(
    (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  if (cancel_token && cancel_token->load()) {
    return;
  }

  // Parse WAV header
  speak_ros::wav_utils::WavHeader header;
  try {
    header = speak_ros::wav_utils::parseWavHeader(wav_data);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Failed to parse WAV header: ") + e.what());
  }

  // Extract PCM data
  const uint8_t * pcm_data = wav_data.data() + header.data_offset;
  const size_t pcm_size = header.data_size;

  // Chunk size (approximately every 100ms)
  const size_t bytes_per_frame = header.channels * (header.bits_per_sample / 8);
  const size_t frames_per_chunk = header.sample_rate / 10;  // 100ms
  const size_t chunk_size = frames_per_chunk * bytes_per_frame;

  // Split PCM data into chunks and send via callback
  size_t offset = 0;
  while (offset < pcm_size) {
    // Check for cancellation
    if (cancel_token && cancel_token->load()) {
      return;
    }

    size_t remaining = pcm_size - offset;
    size_t current_chunk_size = std::min(chunk_size, remaining);

    speak_ros::AudioChunk chunk;
    chunk.data.assign(pcm_data + offset, pcm_data + offset + current_chunk_size);
    chunk.is_final = (offset + current_chunk_size >= pcm_size);

    // Invoke callback (abort if false is returned)
    if (!callback(std::move(chunk))) {
      return;
    }

    offset += current_chunk_size;
  }
}

std::vector<speak_ros::Parameter> open_jtalk_plugin::OpenJTalkPlugin::getParametersDefault() const
{
  return {
    // clang-format off
      {"dictionary_path", "[string] dictionary path", "/var/lib/mecab/dic/open-jtalk/naist-jdic"},
      {"hts_voice_path", "[string] hts voice file path", "/usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice"},
      {"speed_rate", "[string] speed rate", "1.0"}  // clang-format on
  };
}

void open_jtalk_plugin::OpenJTalkPlugin::importParameters(
  const std::unordered_map<std::string, std::variant<int, double, std::string>> & parameters)
{
  dictionary_path = std::get<std::string>(parameters.at("dictionary_path"));
  hts_voice_path = std::get<std::string>(parameters.at("hts_voice_path"));
  speed_rate = std::get<std::string>(parameters.at("speed_rate"));
}
