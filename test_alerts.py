"""
test_alerts.py — Standalone GPIO Alerts Test

Tests a 5V buzzer and a vibration motor via standard GPIO.

Wiring
──────
  Buzzer:
    Positive (Long leg) → Pi Pin 11 (GPIO 17)
    Negative (Short leg) → Pi GND
  
  Vibration Motor (via 2N2222 transistor):
    Motor +       → Pi 5V
    Motor -       → Transistor Collector
    Pi GND        → Transistor Emitter
    Pi GPIO 27    → 1kΩ Resistor → Transistor Base
    (Add flyback diode across motor for safety)

Usage
─────
  python test_alerts.py
"""

import time
import sys

try:
    import RPi.GPIO as GPIO
except ImportError:
    sys.exit("[ERROR] RPi.GPIO is not installed. Run: pip install RPi.GPIO")

BUZZER_PIN = 17
MOTOR_PIN = 27

def main():
    print("=" * 56)
    print("  Local Alerts Test (Buzzer & Motor)")
    print(f"  BUZZER = GPIO {BUZZER_PIN}")
    print(f"  MOTOR  = GPIO {MOTOR_PIN}")
    print("=" * 56)
    print()

    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.setup(MOTOR_PIN, GPIO.OUT)
    
    # Ensure off initially
    GPIO.output(BUZZER_PIN, False)
    GPIO.output(MOTOR_PIN, False)

    try:
        for i in range(1, 4):
            print(f"[INFO] Cycle {i}/3: Alerts ON")
            GPIO.output(BUZZER_PIN, True)
            GPIO.output(MOTOR_PIN, True)
            time.sleep(1.0)
            
            print(f"[INFO] Cycle {i}/3: Alerts OFF")
            GPIO.output(BUZZER_PIN, False)
            GPIO.output(MOTOR_PIN, False)
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        GPIO.cleanup()
        print("[INFO] GPIO cleaned up. Exiting.")

if __name__ == "__main__":
    main()
