"""
test_gsm.py — Standalone GSM Module Test

Tests a GSM module (e.g., SIM900/SIM800) via UART.
Configures SMS text mode and sends a test message.

Wiring
──────
  GSM VCC  → 5V (often requires external power supply depending on module)
  GSM GND  → Pi GND
  GSM TX   → Pi Pin 7  (GPIO 4  / UART3 RXD)
  GSM RX   → Pi Pin 29 (GPIO 5  / UART3 TXD)

Note: This uses Hardware UART3. Connect via /dev/ttyAMA1.
You MUST enable uart3 on the Pi 4 via /boot/config.txt
(add `dtoverlay=uart3`).

Usage
─────
  python test_gsm.py --port /dev/ttyAMA1 --phone "+1234567890"
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("[ERROR] pyserial is not installed. Run: pip install pyserial")

def send_at_command(ser: serial.Serial, command: str, wait: float = 1.0) -> str:
    """Send an AT command and read the response."""
    print(f"  --> {command}")
    ser.write((command + '\r\n').encode('ascii'))
    time.sleep(wait)
    
    response = ""
    while ser.in_waiting > 0:
        response += ser.read(ser.in_waiting).decode('ascii', errors='ignore')
    
    if response:
        print(f"  <-- {response.strip()}")
    return response

def main():
    parser = argparse.ArgumentParser(description="GSM SMS Test")
    parser.add_argument("--port", type=str, default="/dev/ttyAMA1",
                        help="Serial port for GSM (default: /dev/ttyAMA1)")
    parser.add_argument("--baud", type=int, default=9600,
                        help="Baud rate (default: 9600)")
    parser.add_argument("--phone", type=str, required=True,
                        help="Destination phone number (e.g. +1234567890)")
    args = parser.parse_args()

    print("=" * 56)
    print("  GSM Module SMS Test")
    print(f"  UART: {args.port} @ {args.baud} baud")
    print(f"  Dest: {args.phone}")
    print("=" * 56)
    print()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as exc:
        sys.exit(f"[ERROR] Could not open {args.port}: {exc}")

    print(f"[OK]   Serial port {args.port} opened")
    print("[INFO] Initializing module...")

    # Check basic communication
    send_at_command(ser, "AT")
    
    # Check SIM registration
    send_at_command(ser, "AT+CPIN?")
    send_at_command(ser, "AT+CREG?")
    
    # Set SMS to Text Mode
    send_at_command(ser, "AT+CMGF=1")

    print("\n[INFO] Sending SMS...")
    
    # Send recipient number
    ser.write((f'AT+CMGS="{args.phone}"\r\n').encode('ascii'))
    time.sleep(1) # Wait for '>' prompt
    
    # Send message body
    message = "Test SMS from Walking Stick project!"
    ser.write(message.encode('ascii'))
    
    # Send Ctrl+Z (ASCII 26) to indicate end of message
    ser.write(bytes([26]))
    
    print("  --> [Message body sent, waiting for confirmation...]")
    time.sleep(4) # SMS can take a few seconds to send
    
    response = ""
    while ser.in_waiting > 0:
        response += ser.read(ser.in_waiting).decode('ascii', errors='ignore')
    print(f"  <-- {response.strip()}")

    ser.close()
    print("\n[INFO] Test complete. Serial port closed.")

if __name__ == "__main__":
    main()
