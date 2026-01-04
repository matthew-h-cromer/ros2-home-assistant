# Claude Code Project Notes

## Project Overview

ROS2-based home assistant with wake word "Hey Ross".

## Build Commands

```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select assistant

# Source workspace
source install/setup.bash
```

## Key Design Decisions

- **ROS2 Kilted** - Latest LTS distribution (2025)
- **ament_python** - Python-based ROS2 packages (no C++ for now)
- **Offline-first** - Speech recognition and TTS work without internet
- **Modular nodes** - Separate nodes for speech, LLM, TTS for flexibility

## Architecture

- Speech recognition: Port from ros2-learning/src/speech_recognition
- TTS: Piper (offline, fast, good quality)
- LLM: TBD (likely API-based initially)
- Wake word: "Hey Ross" (implementation TBD)

## File Locations

- Main package: `src/assistant/`
- Models cache: `models/` (gitignored)
- Reference implementation: `../ros2-learning/src/speech_recognition/`
