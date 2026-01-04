# Claude Code Project Notes

## Build Commands

```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select assistant

# Source workspace
source install/setup.bash
```

## File Locations

- Main package: `src/assistant/`
- Models cache: `models/` (gitignored)

## Nodes

- `speech_node` - Captures audio, runs VAD, transcribes with Whisper, publishes to `/speech`
- `trigger_node` - Subscribes to `/speech`, detects "Hey Ross", publishes to `/user_requests`
- `llm_node` - Subscribes to `/user_requests`, calls Claude API, publishes to `/assistant_response`

## Environment

API keys are stored in `.env` (gitignored):
```
ANTHROPIC_API_KEY=sk-ant-...
```
