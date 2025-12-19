#!/usr/bin/env python3
"""Quick test script to capture serial output from ESP32"""
import serial
import time
import sys

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
TIMEOUT = 20  # seconds

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
    print(f"Capturing output for {TIMEOUT} seconds...\n")
    print("=" * 60)
    
    start_time = time.time()
    line_count = 0
    
    while (time.time() - start_time) < TIMEOUT:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
                line_count += 1
                # Look for key diagnostic messages
                if '[STATUS]' in line or '[TIMER]' in line or '[DIAG' in line:
                    print("  <-- KEY MESSAGE")
        time.sleep(0.01)
    
    print("=" * 60)
    print(f"\nCaptured {line_count} lines")
    ser.close()
    
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)
except KeyboardInterrupt:
    print("\n\nInterrupted by user")
    ser.close()
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)

