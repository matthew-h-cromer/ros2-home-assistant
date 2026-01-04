# ROS2 Home Assistant

A voice-controlled home assistant powered by ROS2 Kilted. Say **"Hey Ross"** to wake it up.

## Project Goals

- **Short-term:** Conversational AI that answers questions via voice
- **Long-term:** Full home automation integration

### Features (Planned)

- Wake word activation ("Hey Ross")
- Offline speech recognition (Whisper)
- Offline text-to-speech
- LLM integration for natural conversations
- Home automation control (future)

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
- Offline TTS (TBD - likely Piper)

## Project Structure

```
ros2-home-assistant/
├── src/
│   └── assistant/          # Main ROS2 package
├── models/                  # ML models (gitignored)
├── setup.sh                 # Initial setup script
├── Makefile                 # Build automation
└── requirements.txt         # Python dependencies
```

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
# Run the assistant (coming soon)
ros2 run assistant assistant_node
```

## Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Microphone    │────▶│  Speech Node    │────▶│   LLM Node      │
│   (USB Input)   │     │  (Whisper+VAD)  │     │   (TBD)         │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
┌─────────────────┐     ┌─────────────────┐              │
│    Speaker      │◀────│   TTS Node      │◀─────────────┘
│   (USB Output)  │     │   (Piper?)      │
└─────────────────┘     └─────────────────┘
```

### ROS2 Topics (Planned)

| Topic | Type | Description |
|-------|------|-------------|
| `/speech` | `std_msgs/String` | Transcribed speech from user |
| `/response` | `std_msgs/String` | LLM response text |
| `/tts` | `std_msgs/String` | Text to speak |

## Related Projects

- [ros2-learning](../ros2-learning) - Learning project with working speech recognition node

## License

MIT
