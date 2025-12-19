#!/bin/bash
# Script to capture serial output from ESP32 and extract NDJSON debug logs
# Usage: ./capture_debug_logs.sh [serial_port] [duration_seconds]

SERIAL_PORT=${1:-/dev/ttyUSB0}
DURATION=${2:-15}
LOG_FILE="/home/yunusdanabas/as_robotics/.cursor/debug.log"
RAW_OUTPUT="/tmp/esp32_serial_raw.txt"

echo "Capturing serial output from $SERIAL_PORT for $DURATION seconds..."
echo "Extracting NDJSON logs to $LOG_FILE"
echo "Raw output saved to $RAW_OUTPUT"
echo ""

# Clear the log files
> "$LOG_FILE"
> "$RAW_OUTPUT"

# Capture serial output - save both raw and filtered
echo "Starting capture (press Ctrl+C to stop early)..."
timeout $DURATION pio device monitor --port "$SERIAL_PORT" --baud 115200 2>&1 | tee "$RAW_OUTPUT" | \
  grep -E '^\s*\{' | \
  while IFS= read -r line; do
    echo "$line" >> "$LOG_FILE"
  done

echo ""
echo "Capture complete."
echo "Found $(wc -l < "$LOG_FILE") NDJSON log entries in $LOG_FILE"
echo "Raw output: $RAW_OUTPUT ($(wc -l < "$RAW_OUTPUT") lines)"
echo ""
echo "First 20 lines of raw output:"
head -20 "$RAW_OUTPUT"

