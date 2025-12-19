#!/bin/bash
# Helper script to find ESP32 serial ports

echo "Searching for ESP32 devices..."
echo ""

# Ensure glob patterns that match nothing expand to nothing instead of themselves
shopt -s nullglob

# Find USB serial devices
if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
    echo "Found USB-to-Serial devices:"
    for port in /dev/ttyUSB*; do
        echo "  - $port"
        # Try to get device info if possible
        if [ -e "/sys/class/tty/$(basename $port)/device/../../product" ]; then
            product=$(cat "/sys/class/tty/$(basename $port)/device/../../product" 2>/dev/null)
            echo "    Product: $product"
        fi
    done
    echo ""
fi

# Find ACM devices (might be used by some ESP32 boards)
if ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "Found ACM devices:"
    for port in /dev/ttyACM*; do
        echo "  - $port"
    done
    echo ""
fi

# Check permissions
echo "Checking permissions..."
for port in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$port" ]; then
        if [ -r "$port" ] && [ -w "$port" ]; then
            echo "  ✓ $port - Read/Write access OK"
        else
            echo "  ✗ $port - No access (run: sudo chmod 666 $port)"
        fi
    fi
done

echo ""
echo "Current user groups:"
groups | tr ' ' '\n' | grep -E "dialout|uucp|tty" && echo "  ✓ You are in the dialout group" || echo "  ✗ Not in dialout group (run: sudo usermod -a -G dialout \$USER)"

echo ""
echo "To use PlatformIO:"
echo "  pio device list           # List all devices with details"
echo "  pio device monitor        # Monitor serial output"
echo ""

