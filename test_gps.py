"""
test_gps.py — Standalone GPS Module Test

Reads NMEA sentences from a standard GPS module connected via UART
serial on a Raspberry Pi and prints parsed latitude / longitude.

Wiring
──────
  GPS VCC  → Pi Pin 1  (3.3 V)  or Pin 2/4 (5 V) — check your module
  GPS GND  → Pi Pin 6  (GND)
  GPS TX   → Pi Pin 10 (GPIO 15 / UART0 RXD)  — GPS TX  → Pi RX
  GPS RX   → Pi Pin 8  (GPIO 14 / UART0 TXD)  — GPS RX  → Pi TX (optional)

Prerequisites
─────────────
  1. Enable UART via raspi-config:
       sudo raspi-config  →  Interface Options  →  Serial Port
         "Login shell over serial?"  →  No
         "Serial port hardware?"     →  Yes

  2. On Pi 3/4/5, /dev/serial0 is the primary UART.
     If your module uses a different baud rate, adjust BAUD_RATE below.

  3. Install pyserial:
       pip install pyserial

Usage
─────
  python test_gps.py
  python test_gps.py --port /dev/ttyAMA0 --baud 115200
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

try:
    import serial
except ImportError:
    sys.exit(
        "[ERROR] pyserial is not installed.\n"
        "        Install it with:\n"
        "          pip install pyserial"
    )

DEFAULT_PORT = "/dev/serial0"   # symlink to active UART on most Pi models
DEFAULT_BAUD = 9600             # most GPS modules default to 9600

_running = True

def _signal_handler(sig, frame):
    global _running
    _running = False

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

def _nmea_checksum_ok(sentence: str) -> bool:
    """Validate the NMEA XOR checksum (*HH at the end)."""
    if "*" not in sentence:
        return False
    body, cksum_hex = sentence.rsplit("*", 1)
    body = body.lstrip("$")
    computed = 0
    for ch in body:
        computed ^= ord(ch)
    try:
        expected = int(cksum_hex[:2], 16)
    except ValueError:
        return False
    return computed == expected

def _nmea_coord_to_decimal(raw: str, hemisphere: str) -> float | None:
    """
    Convert an NMEA coordinate string to decimal degrees.

    NMEA format:
      Latitude:  ddmm.mmmm   (2-digit degrees)
      Longitude: dddmm.mmmm  (3-digit degrees)
    """
    if not raw or not hemisphere:
        return None
    try:
        dot_idx = raw.index(".")
        deg_digits = dot_idx - 2  # minutes are always 2 digits before the dot
        degrees = float(raw[:deg_digits])
        minutes = float(raw[deg_digits:])
        decimal = degrees + minutes / 60.0
        if hemisphere in ("S", "W"):
            decimal = -decimal
        return decimal
    except (ValueError, IndexError):
        return None

def parse_gga(fields: list[str]) -> dict | None:
    """
    Parse a $GPGGA / $GNGGA sentence.

    Returns a dict with lat, lon, fix_quality, num_satellites, altitude
    or None if there is no valid fix.
    """
    if len(fields) < 10:
        return None

    fix_quality = int(fields[6]) if fields[6] else 0
    if fix_quality == 0:
        return None  # no fix

    lat = _nmea_coord_to_decimal(fields[2], fields[3])
    lon = _nmea_coord_to_decimal(fields[4], fields[5])
    if lat is None or lon is None:
        return None

    return {
        "utc_time": fields[1],
        "latitude": lat,
        "longitude": lon,
        "fix_quality": fix_quality,
        "satellites": int(fields[7]) if fields[7] else 0,
        "altitude_m": float(fields[9]) if fields[9] else 0.0,
    }

def parse_rmc(fields: list[str]) -> dict | None:
    """
    Parse a $GPRMC / $GNRMC sentence.

    Returns a dict with lat, lon, speed_knots, course
    or None if the fix is not active.
    """
    if len(fields) < 10:
        return None

    status = fields[2]   # A = active, V = void
    if status != "A":
        return None

    lat = _nmea_coord_to_decimal(fields[3], fields[4])
    lon = _nmea_coord_to_decimal(fields[5], fields[6])
    if lat is None or lon is None:
        return None

    return {
        "utc_time": fields[1],
        "latitude": lat,
        "longitude": lon,
        "speed_knots": float(fields[7]) if fields[7] else 0.0,
        "course": float(fields[8]) if fields[8] else 0.0,
        "date": fields[9],
    }

def main() -> None:
    parser = argparse.ArgumentParser(description="GPS NMEA Serial Test")
    parser.add_argument("--port", type=str, default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    args = parser.parse_args()

    print("=" * 56)
    print("  GPS Module Test")
    print(f"  UART: {args.port} @ {args.baud} baud")
    print("=" * 56)
    print()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as exc:
        sys.exit(f"[ERROR] Could not open {args.port}: {exc}")

    print(f"[OK]   Serial port {args.port} opened")
    print("[INFO] Waiting for GPS fix — this may take 30-60 s outdoors.")
    print("[INFO] Press Ctrl-C to stop.\n")

    last_lat: float | None = None
    last_lon: float | None = None
    sentence_count = 0

    try:
        while _running:
            raw_line = ser.readline()
            if not raw_line:
                continue

            try:
                line = raw_line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue

            if not line.startswith("$"):
                continue

            sentence_count += 1

            if not _nmea_checksum_ok(line):
                continue

            fields = line.split(",")
            msg_type = fields[0]

            parsed = None

            if msg_type in ("$GPGGA", "$GNGGA"):
                parsed = parse_gga(fields)
            elif msg_type in ("$GPRMC", "$GNRMC"):
                parsed = parse_rmc(fields)

            if parsed is not None:
                lat = parsed["latitude"]
                lon = parsed["longitude"]

                if lat != last_lat or lon != last_lon:
                    last_lat, last_lon = lat, lon
                    print(
                        f"[GPS]  Lat: {lat:+11.6f}°  Lon: {lon:+12.6f}°  "
                        f"(Sats: {parsed.get('satellites', '?')}  "
                        f"Alt: {parsed.get('altitude_m', '?')} m  "
                        f"UTC: {parsed.get('utc_time', '?')})"
                    )
            else:
                if sentence_count % 20 == 0:
                    print(
                        f"  … received {sentence_count} NMEA sentences "
                        f"(no valid fix yet)"
                    )

    finally:
        ser.close()
        print("\n[INFO] Serial port closed. Exiting.")

if __name__ == "__main__":
    main()
