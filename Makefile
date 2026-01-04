.PHONY: build clean rebuild run

# Build all packages
build:
	colcon build --symlink-install

# Clean build artifacts
clean:
	rm -rf build/ install/ log/ models/

# Rebuild from scratch
rebuild: clean build

# Run the assistant (placeholder)
run:
	ros2 run assistant assistant_node
