# ROS2 Home Assistant

A voice-controlled home assistant powered by ROS2 Kilted.

## Hardware

This project is designed to run on a Raspberry Pi 5. Here's the complete hardware list:

| Component | Product | Notes |
|-----------|---------|-------|
| **Compute** | Raspberry Pi 5 8GB | Main processing unit |
| **Cooling** | GeeekPi Active Cooler - Armor Lite V5 | Aluminum heatsink + cooling fan for Raspberry Pi 5 |
| **Microphone** | DUNGZDUZ USB Microphone | Plug-and-play, high sensitivity, cordless mini-sized portable |
| **Speaker** | USB Computer/Laptop Speaker | Stereo sound with enhanced bass, portable mini sound bar |

### Hardware Setup Notes

1. **Cooling:** The Armor Lite V5 cooler attaches directly to the Pi 5 and provides active cooling via a PWM-controlled fan. Essential for sustained voice processing workloads.

2. **Audio Input:** The DUNGZDUZ USB microphone is plug-and-play on Linux. No drivers required.

3. **Audio Output:** The USB speaker works out of the box. For best results, configure it as the default audio output in PulseAudio/PipeWire.

## Software Requirements

- **OS:** Ubuntu 24.04 (or compatible)
- **ROS2:** Kilted Kaiju (2025)
- **Python:** 3.12+

### Dependencies

See `requirements.txt` for Python dependencies. Key components:

- `faster-whisper` - Offline speech recognition
- `silero-vad` - Voice activity detection
- `sounddevice` - Audio capture

## Getting Started

### Prerequisites

1. Install ROS2 Kilted: https://docs.ros.org/en/kilted/Installation.html
2. Install system dependencies:
   ```bash
   sudo apt install portaudio19-dev
   ```

### Setup

```bash
# Clone the repository
git clone <repo-url>
cd ros2-home-assistant

# Run setup script
./setup.sh

# Source the workspace
source install/setup.bash
```

### Running

```bash
# Run all nodes via launch file
make run

# Or directly:
ros2 launch assistant assistant.launch.py
```

To run nodes individually:
```bash
ros2 run assistant speech_node
ros2 run assistant trigger_node
```

The first run downloads the Whisper "small" model (~460 MB) to `models/`.

#### Speech Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model` | `small` | Whisper model size (tiny/base/small/medium/large) |
| `device_index` | `-1` | Microphone device (-1 for default) |
| `vad_threshold` | `0.5` | Voice detection sensitivity (0.0-1.0) |
| `vad_silence_ms` | `500` | Silence duration to end utterance (ms) |

Example with custom parameters:
```bash
ros2 run assistant speech_node --ros-args -p model:=base -p vad_threshold:=0.7
```

#### Trigger Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `speech` | Topic to subscribe to for speech input |
| `output_topic` | `user_requests` | Topic to publish triggered requests |
| `trigger_phrases` | `["hey ross"]` | List of trigger phrases to detect |

The trigger node listens for "Hey Ross" (and variations) and publishes the request text (with the trigger phrase stripped) to `/user_requests`.

Example:
```bash
# Listen for triggered requests
ros2 topic echo /user_requests
```

Say "Hey Ross, turn on the lights" and `/user_requests` will receive: "turn on the lights"

## License

MIT
