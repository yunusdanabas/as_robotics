#!/bin/bash
# Helper script to run micro-ROS agent with serial transport
# Usage: ./run_agent_serial.sh [device] [baudrate]

DEVICE=${1:-/dev/ttyUSB0}
BAUDRATE=${2:-115200}

echo "Starting micro-ROS agent..."
echo "Device: $DEVICE"
echo "Baudrate: $BAUDRATE"
echo ""

docker run -it --rm \
    -v /dev:/dev \
    --privileged \
    --net=host \
    microros/micro-ros-agent:jazzy \
    serial --dev $DEVICE -b $BAUDRATE -v6

