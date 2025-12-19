#!/bin/bash
# Helper script to run micro-ROS agent with WiFi (UDP) transport
# Usage: ./run_agent_wifi.sh [port]

PORT=${1:-8888}

echo "Starting micro-ROS agent (WiFi/UDP mode)..."
echo "Port: $PORT"
echo "Make sure ESP32 is configured with this computer's IP address"
echo ""
echo "To find your IP address, run: hostname -I"
echo ""

docker run -it --rm \
    --net=host \
    microros/micro-ros-agent:jazzy \
    udp4 --port $PORT -v6

