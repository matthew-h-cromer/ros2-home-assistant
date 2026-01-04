.PHONY: build clean rebuild run

# Build all packages
build:
	colcon build --symlink-install

# Clean build artifacts
clean:
	rm -rf build/ install/ log/ models/

# Rebuild from scratch
rebuild: clean
	colcon build --symlink-install

# Run the assistant
run:
	ros2 launch assistant assistant.launch.py
