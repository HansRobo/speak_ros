# speak_ros_openai_tts_plugin

A speak_ros plugin that integrates OpenAI's Text-to-Speech API for high-quality speech synthesis.

## Features

- High-quality text-to-speech using OpenAI's TTS models
- Streaming PCM audio output
- Multiple voice options and playback speed control
- Advanced style instructions for `gpt-4o-mini-tts` model

## Setup

### API Key Configuration

You can configure your OpenAI API key in two ways:

1. **Environment Variable** (recommended for development):
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

2. **ROS Parameter** (takes priority if both are set):
   Set the `api_key` parameter when launching the node.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | `"tts-1"` | TTS model to use: `tts-1` (fastest), `tts-1-hd` (high quality), `gpt-4o-mini-tts` (advanced) |
| `voice` | string | `"alloy"` | Voice selection: `alloy`, `echo`, `fable`, `onyx`, `nova`, `shimmer` |
| `speed` | double | `1.0` | Playback speed (0.25 to 4.0) |
| `instructions` | string | `""` | Style instructions for `gpt-4o-mini-tts` model (e.g., "Speak in a cheerful tone") |
| `api_key` | string | `""` | OpenAI API key (overrides environment variable if set) |

## Usage Example

### Using ROS 2 Launch

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speak_ros',
            executable='speak_ros',
            name='speak_ros',
            parameters=[{
                'plugin': 'speak_ros_openai_tts_plugin::OpenAITTSPlugin',
                'model': 'tts-1-hd',
                'voice': 'nova',
                'speed': 1.1,
                # 'api_key': 'your-api-key'  # Optional: use if not using env var
            }]
        )
    ])
```

### Using ros2 run

```bash
# Using environment variable for API key
export OPENAI_API_KEY="your-api-key-here"
ros2 run speak_ros speak_ros --ros-args \
  -p plugin:=speak_ros_openai_tts_plugin::OpenAITTSPlugin \
  -p model:=tts-1-hd \
  -p voice:=nova
```

## Dependencies

- ROS 2 (Humble or later)
- speak_ros
- cpprestsdk (Microsoft C++ REST SDK)

## License

See the main speak_ros package for license information.
