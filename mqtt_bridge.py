"""Adapter between the integrated hardware application and wakatech.py.

This module deliberately delegates envelopes, topics, HMAC signing, command
validation, SQLite persistence, QoS and reconnect behavior to the existing
reference implementation.  It only translates live application state into
that implementation's public queue functions.
"""

from __future__ import annotations

import os
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def utc_timestamp(value: float | str | None = None) -> str:
    if isinstance(value, str):
        return value
    timestamp = time.time() if value is None else value
    return (
        datetime.fromtimestamp(timestamp, timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


class WakatechMqttBridge:
    """Own the embedded lifecycle of the repository's MQTT reference client."""

    def __init__(self) -> None:
        self.enabled = True  # credentials hardcoded in wakatech.py
        self._mqtt: Any = None
        self._thread: threading.Thread | None = None
        self._closed = False

        if not self.enabled:
            return

        # Import only when MQTT is configured. wakatech.py validates all required
        # environment and opens its durable state database during import.
        import wakatech

        self._mqtt = wakatech

    def start(self) -> "WakatechMqttBridge":
        if not self.enabled or self._thread is not None:
            return self
        self._thread = threading.Thread(
            target=self._mqtt.run,
            kwargs={"enable_sample_heartbeat": False},
            name="wakatech-mqtt",
            daemon=True,
        )
        self._thread.start()
        return self

    @property
    def connected(self) -> bool:
        return bool(self.enabled and self._mqtt.CONNECTED.is_set())

    @property
    def counter_epoch(self) -> str:
        return self._mqtt.BOOT_ID if self.enabled else ""

    def settings(self) -> dict[str, Any]:
        if not self.enabled:
            return {
                "version": 0,
                "fallSensitivity": "medium",
                "falseAlarmGraceSeconds": 30,
            }
        return self._mqtt.settings_snapshot()

    def heartbeat(
        self,
        battery_percent: int,
        wifi_rssi: int,
        location: dict[str, Any] | None,
    ) -> str | None:
        if not self.enabled:
            return None
        return self._mqtt.queue_heartbeat(battery_percent, wifi_rssi, location)

    def activity(self, cumulative_steps: int) -> str | None:
        if not self.enabled:
            return None
        return self._mqtt.queue_activity(cumulative_steps, self.counter_epoch)

    def fall(
        self,
        severity: str,
        battery_percent: int,
        location: dict[str, Any],
    ) -> str | None:
        if not self.enabled:
            return None
        return self._mqtt.queue_fall(severity, battery_percent, location)

    def walk(self, started_at: str, ended_at: str, steps: int) -> str | None:
        if not self.enabled:
            return None
        return self._mqtt.queue_walk(started_at, ended_at, steps)

    def stop(self) -> None:
        if not self.enabled or self._closed:
            return
        self._mqtt.request_stop()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._mqtt.STORE.close()
        self._closed = True


def current_wifi_rssi(default: int = -100) -> int:
    """Read Linux wireless signal level without logging network credentials."""
    try:
        lines = Path("/proc/net/wireless").read_text(encoding="ascii").splitlines()[2:]
        for line in lines:
            fields = line.replace(".", "").split()
            if len(fields) >= 4:
                return int(float(fields[3]))
    except (OSError, ValueError):
        pass
    return default


def current_battery_percent() -> int:
    """Read an attached power-supply gauge, then use an explicit safe fallback.

    A bare Raspberry Pi exposes no battery percentage. Installations without a
    UPS/fuel gauge must set WAKATECH_BATTERY_PERCENT to their known test value.
    """
    for capacity in Path("/sys/class/power_supply").glob("*/capacity"):
        try:
            value = int(capacity.read_text(encoding="ascii").strip())
            if 0 <= value <= 100:
                return value
        except (OSError, ValueError):
            continue
    value = int(os.getenv("WAKATECH_BATTERY_PERCENT", "100"))
    if not 0 <= value <= 100:
        raise ValueError("WAKATECH_BATTERY_PERCENT must be between 0 and 100")
    return value


def mqtt_location(
    latitude: float,
    longitude: float,
    fix_at: float | str,
    accuracy_meters: float | None = None,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "latitude": latitude,
        "longitude": longitude,
        "fixAt": utc_timestamp(fix_at),
    }
    if accuracy_meters is not None:
        result["accuracyMeters"] = accuracy_meters
    return result
