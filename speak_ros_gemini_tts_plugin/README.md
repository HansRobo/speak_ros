# speak_ros_gemini_tts_plugin

Gemini text-to-speech through the streaming Interactions API. The plugin converts
inline audio events into the standard speak_ros PCM callback; the shared
`speak_ros_http` package owns HTTP cancellation and PCM frame alignment.

## Usage

Build with `colcon build --packages-up-to speak_ros_gemini_tts_plugin` and source
`install/setup.bash`. Provide a key through `GEMINI_API_KEY` (or the `api_key`
parameter, which takes precedence), then run:

```bash
ros2 run speak_ros speak_ros_node --ros-args \
  -p plugin_name:=gemini_tts_plugin::GeminiTTSPlugin \
  -p gemini_tts_plugin/voice:=Kore
```

Use the normal `/speak` action to request speech. Output is fixed to 24 kHz,
mono, signed 16-bit little-endian PCM. Models must support this Interactions TTS
request format; this is not a Gemini Live API or Vertex AI integration.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `model` | `gemini-3.8-flash-tts` | Interactions-compatible TTS model ID |
| `voice` | `Kore` | Voice accepted by the selected model |
| `style` | empty | Optional speech metadata style, such as `calm and friendly` |
| `api_key` | empty | Overrides `GEMINI_API_KEY` |
| `base_url` | `https://generativelanguage.googleapis.com` | Trusted API origin |

Parameters are under the `gemini_tts_plugin/` namespace. `base_url` must be an
HTTP(S) origin without credentials, path, query, or fragment; requests always use
`/v1beta/interactions`. Only configure trusted endpoints: the key is sent in the
`x-goog-api-key` header. Redirects are disabled and HTTPS certificates are validated.

## Streaming behavior

- SSE parsing supports LF/CRLF, comments, multiline `data`, and arbitrary network
  fragmentation. Individual events are limited to 4 MiB; no whole utterance is
  accumulated. cpprestsdk's internal HTTP buffering is outside this bound.
- Initial `step.start` model-output audio and `step.delta` audio are decoded from
  strict Base64 into frame-aligned PCM. Explicit incompatible rate, channel, or
  MIME metadata fails; URI-based audio is not fetched.
- Normal completion requires an `interaction.completed` event with status
  `completed`, followed by a complete HTTP response. An empty successful stream
  still emits the final marker. Socket EOF alone, protocol errors, incomplete
  frames, and failed interactions never emit a normal completion marker.
- Cancellation interrupts HTTP header/body waits. A rejected PCM callback stops
  the transfer. Failures are not retried, avoiding duplicate speech.
- Requests use `stream: true` and `store: false`. Only single-speaker synthesis is
  exposed; speech style and voice settings stay local to this provider.

The request schema follows the
[official speech guide](https://ai.google.dev/gemini-api/docs/speech-generation)
and the official Python SDK's
[`GenerationConfig.speech_config`](https://github.com/googleapis/python-genai/blob/main/google/genai/_gaos/types/interactions/generationconfig.py).
The REST reference currently labels that field `structured_speech_config`, which
differs from the guide and SDK; this implementation uses `speech_config` without
sending both alternatives. Event handling follows the
[current Interactions streaming schema](https://ai.google.dev/gemini-api/docs/interactions-breaking-changes-may-2026#streaming).

Tests use a delayed local HTTP server and dummy credentials to verify request
shape, early audio, SSE fragmentation, strict decoding, failures, and cancellation.
A live Gemini API call and actual audio playback have not been validated.
