#!/bin/bash

# Build the Docker image
docker-compose build

# Run the Docker container
docker-compose up -d

# Wait a moment for container to start
sleep 2

# Attach to the container
docker exec -it kinetirover_dev bash -c "source /opt/ros/noetic/setup.bash && cd /workspace && bash"