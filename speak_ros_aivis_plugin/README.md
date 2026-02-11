# speak_ros_aivis_plugin

A speak_ros plugin for [Aivis Speech Engine](https://github.com/Aivis-Project/AivisSpeech-Engine)

## Overview

Aivis Speech Engine is an open-source speech synthesis engine with VOICEVOX-compatible API. This plugin allows you to use Aivis Speech Engine through speak_ros.

## Features

- VOICEVOX API compatible (`/audio_query` → `/synthesis`)
- Default port: **10101**
- Support for Aivis-specific parameters:
  - `intonationScale`: Emotion intensity (0.0-2.0)
  - `tempoDynamicsScale`: Speaking rate variation (0.0-2.0)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `speaker` | int | `888753760` | Aivis speaker ID |
| `host_name` | string | `"localhost"` | Aivis engine host |
| `port` | int | `10101` | Aivis engine port |
| `speedScale` | double | `1.0` | Speaking rate (1.0 is standard) |
| `pitchScale` | double | `0.0` | Pitch adjustment |
| `intonationScale` | double | `1.0` | Emotion intensity (0.0-2.0) |
| `volumeScale` | double | `1.0` | Volume scale |
| `tempoDynamicsScale` | double | `1.0` | Speaking rate variation (0.0-2.0) |
| `prePhonemeLength` | double | `0.1` | Pre-phoneme length [sec] |
| `postPhonemeLength` | double | `0.1` | Post-phoneme length [sec] |
| `outputSamplingRate` | int | `44100` | Output sampling rate [Hz] |
| `outputStereo` | string | `"false"` | Stereo output |

## Speaker IDs

Default model (Mao v1.2.0):

| ID | Style |
|----|-------|
| 888753760 | Normal |
| 888753761 | Natural |
| 888753762 | Sweet |
| 888753763 | Calm |
| 888753764 | Teasing |
| 888753765 | Sad |

Check available speaker IDs:

```bash
curl http://localhost:10101/speakers
```

## Differences from VOICEVOX

| Item | VOICEVOX | Aivis |
|------|----------|-------|
| Port | `50021` (fixed) | `10101` (configurable) |
| `speedScale` default | `2.0` | `1.0` |
| `intonationScale` | Not available | Available (emotion intensity) |
| `tempoDynamicsScale` | Not available | Available (speaking rate variation) |
| `outputSamplingRate` | `24000` | `44100` |
| `outputStereo` | `"true"` | `"false"` |
