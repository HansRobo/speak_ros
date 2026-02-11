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

#include "speak_ros/audio_player.hpp"

#include <portaudio.h>

#include <condition_variable>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace speak_ros
{

namespace
{

// Convert PortAudio error code to string
std::string paErrorToString(PaError err)
{
  std::ostringstream oss;
  oss << "PortAudio error: " << Pa_GetErrorText(err) << " (code " << err << ")";
  return oss.str();
}

}  // anonymous namespace

AudioPlayer::AudioPlayer()
{
  PaError err = Pa_Initialize();
  if (err != paNoError) {
    throw std::runtime_error("Failed to initialize PortAudio: " + paErrorToString(err));
  }
  initialized_ = true;
}

AudioPlayer::~AudioPlayer()
{
  if (stream_ != nullptr) {
    Pa_CloseStream(stream_);
    stream_ = nullptr;
  }

  if (initialized_) {
    Pa_Terminate();
    initialized_ = false;
  }
}

void AudioPlayer::open(const AudioFormat & format)
{
  // Close existing stream
  if (stream_ != nullptr) {
    Pa_CloseStream(stream_);
    stream_ = nullptr;
  }

  format_ = format;
  is_done_.store(false);

  // Determine sample format
  PaSampleFormat sample_format;
  if (format.bits_per_sample == 16) {
    sample_format = paInt16;
  } else if (format.bits_per_sample == 8) {
    sample_format = paInt8;
  } else if (format.bits_per_sample == 32) {
    sample_format = paInt32;
  } else {
    throw std::runtime_error(
      "Unsupported bits_per_sample: " + std::to_string(format.bits_per_sample));
  }

  // Open stream with default output device
  PaError err = Pa_OpenDefaultStream(
    &stream_,
    0,                             // Input channels (no recording)
    format.channels,               // Output channels
    sample_format,                 // Sample format
    format.sample_rate,            // Sampling rate
    paFramesPerBufferUnspecified,  // Frames per buffer (auto)
    nullptr,                       // No callback (blocking I/O)
    nullptr                        // No user data
  );

  if (err != paNoError) {
    throw std::runtime_error("Failed to open PortAudio stream: " + paErrorToString(err));
  }

  // Start stream
  err = Pa_StartStream(stream_);
  if (err != paNoError) {
    Pa_CloseStream(stream_);
    stream_ = nullptr;
    throw std::runtime_error("Failed to start PortAudio stream: " + paErrorToString(err));
  }
}

void AudioPlayer::play(
  AudioQueue & queue, CancelToken cancel_token, std::atomic<uint64_t> & played_samples)
{
  if (stream_ == nullptr) {
    throw std::runtime_error("AudioPlayer::play() called before open()");
  }

  played_samples.store(0);

  while (true) {
    // Check for cancellation
    if (cancel_token && cancel_token->load()) {
      Pa_AbortStream(stream_);  // Stop immediately (discard buffer)
      is_done_.store(true);
      return;
    }

    // Get chunk from queue
    std::optional<AudioChunk> chunk = queue.pop(cancel_token);

    // Queue finished or cancelled
    if (!chunk.has_value()) {
      break;
    }

    // Play PCM data
    if (!chunk->data.empty()) {
      size_t frames = chunk->data.size() / format_.bytesPerFrame();
      PaError err = Pa_WriteStream(stream_, chunk->data.data(), frames);

      if (err != paNoError) {
        // Stop on error
        Pa_AbortStream(stream_);
        is_done_.store(true);
        throw std::runtime_error("Failed to write to PortAudio stream: " + paErrorToString(err));
      }

      played_samples.fetch_add(frames);
    }

    // Exit if this is the final chunk
    if (chunk->is_final) {
      break;
    }
  }

  // On normal completion, stop after playing remaining buffer
  PaError err = Pa_StopStream(stream_);
  if (err != paNoError) {
    throw std::runtime_error("Failed to stop PortAudio stream: " + paErrorToString(err));
  }

  is_done_.store(true);
}

void AudioPlayer::stop()
{
  if (stream_ != nullptr) {
    Pa_AbortStream(stream_);  // Stop immediately
  }
  is_done_.store(true);
}

void AudioPlayer::close()
{
  if (stream_ != nullptr) {
    Pa_CloseStream(stream_);
    stream_ = nullptr;
  }
}

bool AudioPlayer::waitForDone(std::chrono::milliseconds timeout)
{
  auto start = std::chrono::steady_clock::now();

  while (!is_done_.load()) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed >= timeout) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return true;
}

bool AudioPlayer::isDone() const { return is_done_.load(); }

}  // namespace speak_ros
