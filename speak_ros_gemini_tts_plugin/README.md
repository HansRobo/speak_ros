# speak_ros_gemini_tts_plugin

Single-speaker streaming TTS through the Gemini Interactions API, using
`speak_ros` HTTP/PCM helpers. Output is 24 kHz mono signed 16-bit little-endian PCM.

Build with `colcon build --packages-up-to speak_ros_gemini_tts_plugin`, source
`install/setup.bash`, set `GEMINI_API_KEY`, then run:

```bash
ros2 run speak_ros speak_ros_node --ros-args \
  -p plugin_name:=gemini_tts_plugin::GeminiTTSPlugin
```

Request speech through the normal `/speak` action.
Parameters use the `gemini_tts_plugin/` namespace:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `model` | `gemini-3.8-flash-tts` | Interactions-compatible TTS model |
| `voice` | `Kore` | Voice accepted by the model |
| `style` | empty | Optional speech style |
| `api_key` | empty | Overrides `GEMINI_API_KEY` |
| `base_url` | `https://generativelanguage.googleapis.com` | Trusted HTTP(S) origin; receives the API key |

Requests use `/v1beta/interactions`, `stream: true`, and `store: false`.
Normal completion requires a successful API completion event and HTTP EOF.

The request uses `speech_config` from the [speech guide](https://ai.google.dev/gemini-api/docs/speech-generation)
and [official SDK](https://github.com/googleapis/python-genai/blob/main/google/genai/_gaos/types/interactions/generationconfig.py);
the REST reference labels it `structured_speech_config`.
Tests use a local mock server. Live API calls and audio playback remain unverified.
