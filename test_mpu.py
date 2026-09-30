"""
test_mpu.py — Standalone MPU-6050 Sensor Test

Tests the MPU-6050 accelerometer/gyroscope via I2C on a Raspberry Pi.

Wiring
──────
  MPU-6050 VCC  → Pi Pin 1  (3.3 V)
  MPU-6050 GND  → Pi Pin 6  (GND)
  MPU-6050 SDA  → Pi Pin 3  (GPIO 2 / I2C1 SDA)
  MPU-6050 SCL  → Pi Pin 5  (GPIO 3 / I2C1 SCL)

Prerequisites
─────────────
  sudo raspi-config          # → Interface Options → I2C → Enable
  sudo apt install -y i2c-tools python3-smbus
  pip install smbus2

  Verify the sensor is visible:
    sudo i2cdetect -y N      # replace N with the adapter reported by i2cdetect -l

Usage
─────
  python test_mpu.py
"""

from __future__ import annotations

import signal
import struct
import sys
import time
from pathlib import Path

try:
    import smbus2
except ImportError:
    sys.exit(
        "[ERROR] smbus2 is not installed.\n"
        "        Install it with:\n"
        "          pip install smbus2\n"
        "        (Also ensure I2C is enabled via raspi-config.)"
    )

MPU6050_ADDR        = 0x68   # default I2C address (AD0 → GND)
# The bus is detected automatically; use --i2c-bus N to override it.
I2C_BUS             = 1

REG_PWR_MGMT_1      = 0x6B   # power management — write 0x00 to wake up
REG_WHO_AM_I        = 0x75   # should return 0x68
REG_ACCEL_XOUT_H    = 0x3B   # start of 14-byte burst (accel + temp + gyro)

ACCEL_SCALE = 16384.0   # LSB/g   for ±2 g  range
GYRO_SCALE  = 131.0     # LSB/°/s for ±250 °/s range

FALL_ACCEL_THRESHOLD = 2.5   # g — adjust to suit real-world testing

_running = True

def _signal_handler(sig, frame):
    global _running
    _running = False

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

def mpu_init(bus: smbus2.SMBus) -> None:
    """Wake the MPU-6050 by clearing its sleep bit."""
    bus.write_byte_data(MPU6050_ADDR, REG_PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    who = bus.read_byte_data(MPU6050_ADDR, REG_WHO_AM_I)
    if who != 0x68:
        print(f"[WARN] WHO_AM_I returned 0x{who:02X} (expected 0x68)")
    else:
        print(f"[OK]   MPU-6050 detected (WHO_AM_I = 0x{who:02X})")

def mpu_read_raw(bus: smbus2.SMBus) -> tuple[float, float, float, float, float, float, float]:
    """
    Burst-read 14 bytes starting at ACCEL_XOUT_H.

    Returns
    -------
    ax, ay, az   — accelerometer in g
    temp         — die temperature in °C
    gx, gy, gz   — gyroscope in °/s
    """
    data = bus.read_i2c_block_data(MPU6050_ADDR, REG_ACCEL_XOUT_H, 14)

    raw = struct.unpack(">hhhhhhh", bytes(data))

    ax = raw[0] / ACCEL_SCALE
    ay = raw[1] / ACCEL_SCALE
    az = raw[2] / ACCEL_SCALE
    temp = (raw[3] / 340.0) + 36.53
    gx = raw[4] / GYRO_SCALE
    gy = raw[5] / GYRO_SCALE
    gz = raw[6] / GYRO_SCALE

    return ax, ay, az, temp, gx, gy, gz


def open_mpu_bus(bus_setting: str):
    """Open the configured bus, or find the bus whose MPU identifies correctly."""
    if bus_setting.lower() != "auto":
        bus_number = int(bus_setting)
        return smbus2.SMBus(bus_number), bus_number

    candidates = sorted(
        int(path.name.rsplit("-", 1)[1])
        for path in Path("/dev").glob("i2c-*")
        if path.name.rsplit("-", 1)[-1].isdigit()
    )
    for bus_number in candidates:
        bus = None
        found = False
        try:
            bus = smbus2.SMBus(bus_number)
            if bus.read_byte_data(MPU6050_ADDR, REG_WHO_AM_I) in (0x68, 0x69):
                found = True
                return bus, bus_number
        except Exception:
            pass
        finally:
            if bus is not None and not found:
                bus.close()
    raise FileNotFoundError(
        "MPU-6050 was not found on any I2C adapter. "
        f"Checked: {', '.join(f'/dev/i2c-{number}' for number in candidates) or 'none'}."
    )

def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="MPU-6050 sensor test")
    parser.add_argument("--i2c-bus", default="auto", help="I2C bus number or auto (default: auto)")
    args = parser.parse_args()

    print("=" * 56)
    print("  MPU-6050 Sensor Test")
    print("  (I2C bus auto-detect — Pins 3 / 5)")
    print("=" * 56)
    print()

    try:
        bus, bus_number = open_mpu_bus(args.i2c_bus)
    except (FileNotFoundError, ValueError, OSError) as exc:
        sys.exit(f"[ERROR] {exc}\n        Check wiring, enable I2C, or pass --i2c-bus N.")
    print(f"[INFO] Using /dev/i2c-{bus_number}.")
    mpu_init(bus)

    print(f"[INFO] Fall detection threshold: {FALL_ACCEL_THRESHOLD} g")
    print("[INFO] Streaming data — press Ctrl-C to stop.\n")

    sample = 0

    try:
        while _running:
            ax, ay, az, temp, gx, gy, gz = mpu_read_raw(bus)
            sample += 1

            accel_mag = (ax**2 + ay**2 + az**2) ** 0.5

            print(
                f"[{sample:>6d}]  "
                f"Accel(g)  X={ax:+7.3f}  Y={ay:+7.3f}  Z={az:+7.3f}  |  "
                f"Gyro(°/s)  X={gx:+8.2f}  Y={gy:+8.2f}  Z={gz:+8.2f}  |  "
                f"Temp={temp:5.1f}°C  |  |A|={accel_mag:.3f} g"
            )

            if accel_mag > FALL_ACCEL_THRESHOLD:
                print(
                    f"  *** FALL DETECTED ***  "
                    f"|Accel| = {accel_mag:.2f} g  (threshold = {FALL_ACCEL_THRESHOLD} g)"
                )

            time.sleep(0.1)  # ~10 Hz polling

    finally:
        bus.close()
        print("\n[INFO] I2C bus closed. Exiting.")

if __name__ == "__main__":
    main()
