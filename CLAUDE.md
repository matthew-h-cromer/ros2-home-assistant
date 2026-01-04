# Claude Code Project Notes

## Build & Run

```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch assistant assistant.launch.py
```

## Environment

Requires `.env` file in repo root (gitignored):
```
ANTHROPIC_API_KEY=sk-ant-...
```

## Architecture

`speech_node` → `/speech` → `wake_node` → `/user_requests` → `llm_node` → `/assistant_response`

- **speech_node**: Audio capture, VAD, Whisper transcription
- **wake_node**: Wake word detection ("Hey Ross"), conversation flow
- **llm_node**: Claude API, conversation history, 30s timeout
