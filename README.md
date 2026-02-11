# speak_ros

A plugin-based text-to-speech system for ROS 2

## Architecture

speak_ros provides a flexible TTS framework built on:

- **Action-based interface** - Asynchronous speech synthesis with progress feedback and cancellation support
- **Plugin architecture** - Switchable TTS engines via pluginlib without code changes
- **Streaming support** - Chunked audio synthesis and playback for low-latency response
- **PortAudio integration** - Cross-platform audio playback

The core package loads TTS plugins dynamically and manages the synthesis pipeline, from text input through audio generation to speaker output.

## Packages

| Package | Description |
|---------|-------------|
| `speak_ros` | Core TTS action server implementation |
| `speak_ros_interfaces` | Custom message, service, and action definitions |
| `speak_ros_open_jtalk_plugin` | Open JTalk TTS plugin (Japanese) |
| `speak_ros_openai_tts_plugin` | OpenAI TTS API plugin |
| `speak_ros_voicevox_plugin` | VOICEVOX TTS plugin (Japanese) |

## Available Plugins

- **Open JTalk** - Offline Japanese speech synthesis
- **OpenAI TTS** - High-quality cloud-based synthesis via OpenAI API
- **VOICEVOX** - Japanese synthesis with diverse character voices

## Building

### Prerequisites

- ROS 2 (Humble or later)
- PortAudio development libraries

### Install Dependencies

```bash
cd <your_ros2_workspace>
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
colcon build --packages-up-to speak_ros speak_ros_open_jtalk_plugin speak_ros_openai_tts_plugin speak_ros_voicevox_plugin
source install/setup.bash
```

## Usage

### Running the TTS Server

Start the TTS action server with the default plugin (Open JTalk):

```bash
ros2 run speak_ros speak_ros_node
```

To use a different plugin, specify the `plugin_name` parameter:

```bash
ros2 run speak_ros speak_ros_node --ros-args -p plugin_name:=speak_ros_openai_tts_plugin::OpenAITTSPlugin
```

### Testing with the Test Client

Use the provided test client to send text for synthesis:

```bash
# In another terminal
ros2 run speak_ros test_client
```

The test client sends a sample text to the `/speak` action server and displays feedback during synthesis and playback.

### Listing Available Plugins

To see all installed TTS plugins:

```bash
ros2 run speak_ros list_plugins
```

### Action Interface

The `/speak` action accepts text and optional parameter overrides:

```bash
ros2 action send_goal /speak speak_ros_interfaces/action/Speak "{text: 'Hello world'}"
```

## License

Apache 2.0
