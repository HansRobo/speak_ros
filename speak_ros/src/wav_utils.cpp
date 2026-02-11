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

#include "speak_ros/wav_utils.hpp"

#include <cstring>
#include <sstream>

namespace speak_ros
{
namespace wav_utils
{

namespace
{

// Read 4-byte unsigned integer in little-endian format
uint32_t readUint32LE(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

// Read 2-byte unsigned integer in little-endian format
uint16_t readUint16LE(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

// Compare 4-character chunk ID
bool matchChunkId(const uint8_t * data, const char * id) { return std::memcmp(data, id, 4) == 0; }

}  // anonymous namespace

WavHeader parseWavHeader(const std::vector<uint8_t> & wav_data)
{
  if (wav_data.size() < 44) {
    throw std::runtime_error("WAV data too short (minimum 44 bytes required)");
  }

  const uint8_t * data = wav_data.data();

  // Check RIFF header
  if (!matchChunkId(data, "RIFF")) {
    throw std::runtime_error("Invalid WAV file: missing RIFF header");
  }

  // Check WAVE format
  if (!matchChunkId(data + 8, "WAVE")) {
    throw std::runtime_error("Invalid WAV file: missing WAVE format");
  }

  // Search for fmt chunk
  size_t offset = 12;
  bool fmt_found = false;
  WavHeader header = {};

  while (offset + 8 <= wav_data.size()) {
    if (matchChunkId(data + offset, "fmt ")) {
      uint32_t chunk_size = readUint32LE(data + offset + 4);
      if (offset + 8 + chunk_size > wav_data.size()) {
        throw std::runtime_error("Invalid WAV file: fmt chunk size exceeds file size");
      }

      if (chunk_size < 16) {
        throw std::runtime_error("Invalid WAV file: fmt chunk too small");
      }

      uint16_t audio_format = readUint16LE(data + offset + 8);
      if (audio_format != 1) {  // 1 = PCM
        throw std::runtime_error("Unsupported WAV format: only PCM (format 1) is supported");
      }

      header.channels = readUint16LE(data + offset + 10);
      header.sample_rate = readUint32LE(data + offset + 12);
      header.bits_per_sample = readUint16LE(data + offset + 22);

      fmt_found = true;
      offset += 8 + chunk_size;
      break;
    } else {
      // Skip other chunks
      uint32_t chunk_size = readUint32LE(data + offset + 4);
      offset += 8 + chunk_size;
    }
  }

  if (!fmt_found) {
    throw std::runtime_error("Invalid WAV file: fmt chunk not found");
  }

  // Search for data chunk
  bool data_found = false;
  while (offset + 8 <= wav_data.size()) {
    if (matchChunkId(data + offset, "data")) {
      uint32_t chunk_size = readUint32LE(data + offset + 4);
      header.data_offset = offset + 8;
      header.data_size = chunk_size;
      data_found = true;
      break;
    } else {
      // Skip other chunks
      uint32_t chunk_size = readUint32LE(data + offset + 4);
      offset += 8 + chunk_size;
    }
  }

  if (!data_found) {
    throw std::runtime_error("Invalid WAV file: data chunk not found");
  }

  if (header.data_offset + header.data_size > wav_data.size()) {
    throw std::runtime_error("Invalid WAV file: data chunk size exceeds file size");
  }

  return header;
}

std::vector<uint8_t> extractPcmData(const std::vector<uint8_t> & wav_data)
{
  WavHeader header = parseWavHeader(wav_data);

  std::vector<uint8_t> pcm_data(
    wav_data.begin() + header.data_offset,
    wav_data.begin() + header.data_offset + header.data_size);

  return pcm_data;
}

AudioFormat toAudioFormat(const WavHeader & header)
{
  AudioFormat format;
  format.sample_rate = header.sample_rate;
  format.channels = header.channels;
  format.bits_per_sample = header.bits_per_sample;
  return format;
}

}  // namespace wav_utils
}  // namespace speak_ros
