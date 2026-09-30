"""
test_ultrasonic.py — Standalone HC-SR04 Test

Tests the ultrasonic distance sensor using Raspberry Pi GPIO.
Continuously calculates and prints distance without blocking.

Wiring
──────
  VCC   → Pi Pin 2/4 (5V)
  GND   → Pi Pin 6   (GND)
  TRIG  → Pi Pin 16  (GPIO 23)
  ECHO  → Pi Pin 18  (GPIO 24) — MUST USE VOLTAGE DIVIDER!
         (Echo is 5V, Pi GPIO is 3.3V max. Use 1kΩ and 2kΩ resistors)

Usage
─────
  python test_ultrasonic.py
"""

import time
import sys

try:
    import RPi.GPIO as GPIO
except ImportError:
    sys.exit("[ERROR] RPi.GPIO is not installed. Run: pip install RPi.GPIO")

# Define GPIO pins (using BCM numbering)
TRIG_PIN = 23
ECHO_PIN = 24

def main():
    print("=" * 56)
    print("  HC-SR04 Ultrasonic Sensor Test")
    print("  TRIG = GPIO 23, ECHO = GPIO 24")
    print("=" * 56)
    print()

    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    # Allow sensor to settle
    GPIO.output(TRIG_PIN, False)
    print("[INFO] Waiting for sensor to settle...")
    time.sleep(2)

    print("[INFO] Starting measurements. Press Ctrl-C to stop.\n")

    try:
        while True:
            # Send 10us pulse to trigger
            GPIO.output(TRIG_PIN, True)
            time.sleep(0.00001)
            GPIO.output(TRIG_PIN, False)

            pulse_start = time.time()
            pulse_end = time.time()
            timeout_start = time.time()

            # Wait for echo to go HIGH
            while GPIO.input(ECHO_PIN) == 0:
                pulse_start = time.time()
                if pulse_start - timeout_start > 0.1: # 100ms timeout
                    break

            # Wait for echo to go LOW
            while GPIO.input(ECHO_PIN) == 1:
                pulse_end = time.time()
                if pulse_end - timeout_start > 0.1: # 100ms timeout
                    break

            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            # Speed of sound is 34300 cm/s
            distance = pulse_duration * 17150
            distance = round(distance, 2)

            if distance < 400: # Valid range is ~2cm to 400cm
                print(f"Distance: {distance:>6.2f} cm")
            else:
                print("Distance: Out of range")

            time.sleep(0.5) # Poll at 2 Hz
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        GPIO.cleanup()
        print("[INFO] GPIO cleaned up. Exiting.")

if __name__ == "__main__":
    main()
