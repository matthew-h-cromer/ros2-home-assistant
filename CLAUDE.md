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
