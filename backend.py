"""Durable, backend-agnostic event delivery for the walking stick."""

from __future__ import annotations

import json
import queue
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class BackendClient:
    """POST JSON events without blocking sensor or camera processing.

    The endpoint can be a Supabase REST insert endpoint, a Firebase HTTPS
    function, or any service accepting the documented event envelope.
    Failed events are retained in a JSONL spool and retried on later starts.
    """

    def __init__(
        self,
        url: str,
        device_id: str,
        spool_path: str | Path,
        token: str = "",
        timeout: float = 10.0,
        max_retries: int = 4,
        location_queue: queue.Queue | None = None,
    ) -> None:
        self.url = url.strip()
        self.device_id = device_id.strip() or "walking-stick"
        self.spool_path = Path(spool_path)
        self.token = token.strip()
        self.timeout = max(1.0, timeout)
        self.max_retries = max(0, max_retries)
        self.location_queue = location_queue
        self._location_requested = False
        self._events: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=64)
        self._running = False
        self._thread: threading.Thread | None = None
        self._sequence = 0
        self._spool_lock = threading.Lock()

    @property
    def enabled(self) -> bool:
        return bool(self.url)

    def start(self) -> "BackendClient":
        if not self.enabled or self._running:
            return self
        self._running = True
        self._thread = threading.Thread(target=self._run, name="backend", daemon=True)
        self._thread.start()
        return self

    def publish(self, event_type: str, payload: dict[str, Any], priority: str = "normal") -> bool:
        """Queue an event; return False if disabled or the queue is full."""
        if not self.enabled:
            return False
        self._sequence += 1
        event = {
            "event_id": str(uuid.uuid4()),
            "device_id": self.device_id,
            "sequence": self._sequence,
            "event_type": event_type,
            "priority": priority,
            "occurred_at": utc_now(),
            "payload": payload,
        }
        try:
            self._events.put_nowait(event)
            return True
        except queue.Full:
            self._spool(event)
            return False

    def request_last_location(self) -> bool:
        """Ask the worker to fetch the latest location for this device."""
        if not self.enabled or self._location_requested:
            return False
        self._location_requested = True
        try:
            self._events.put_nowait({"_request": "last_location"})
            return True
        except queue.Full:
            self._location_requested = False
            return False

    def _fetch_last_location(self) -> None:
        query = urllib.parse.urlencode({
            "device_id": self.device_id,
            "event_type": "telemetry",
            "latest": "1",
        })
        separator = "&" if "?" in self.url else "?"
        request = urllib.request.Request(
            f"{self.url}{separator}{query}",
            headers={"Accept": "application/json"},
            method="GET",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                result = json.loads(response.read().decode("utf-8"))
            if isinstance(result, list):
                result = result[0] if result else {}
            location = result.get("location", result.get("gps", result)) if isinstance(result, dict) else {}
            if isinstance(location, dict) and location.get("latitude") is not None and location.get("longitude") is not None:
                if self.location_queue is not None:
                    self.location_queue.put_nowait({
                        "latitude": float(location["latitude"]),
                        "longitude": float(location["longitude"]),
                        "fix_at": location.get("fix_at") or result.get("occurred_at"),
                        "source": "backend",
                    })
        except Exception as exc:
            print(f"[WARN] Last GPS location unavailable: {exc}")
        finally:
            self._location_requested = False

    def _request(self, event: dict[str, Any]) -> None:
        body = json.dumps(event, separators=(",", ":")).encode("utf-8")
        headers = {"Content-Type": "application/json", "Accept": "application/json"}
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
            headers["apikey"] = self.token
        request = urllib.request.Request(self.url, data=body, headers=headers, method="POST")
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            if response.status < 200 or response.status >= 300:
                raise urllib.error.HTTPError(self.url, response.status, "unexpected status", response.headers, None)

    def _spool(self, event: dict[str, Any]) -> None:
        try:
            self.spool_path.parent.mkdir(parents=True, exist_ok=True)
            with self._spool_lock, self.spool_path.open("a", encoding="utf-8") as spool:
                spool.write(json.dumps(event, separators=(",", ":")) + "\n")
        except OSError as exc:
            print(f"[WARN] Cannot persist backend event: {exc}")

    def _read_spool(self) -> list[dict[str, Any]]:
        if not self.spool_path.exists():
            return []
        try:
            with self._spool_lock, self.spool_path.open(encoding="utf-8") as spool:
                events = [json.loads(line) for line in spool if line.strip()]
            self.spool_path.unlink(missing_ok=True)
            return events
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[WARN] Cannot read backend spool: {exc}")
            return []

    def _deliver(self, event: dict[str, Any]) -> bool:
        for attempt in range(self.max_retries + 1):
            try:
                self._request(event)
                return True
            except Exception as exc:
                if attempt == self.max_retries:
                    print(f"[WARN] Backend delivery failed: {exc}")
                    return False
                time.sleep(min(30.0, 2.0**attempt))
        return False

    def _run(self) -> None:
        pending = self._read_spool()
        while self._running or not self._events.empty():
            if pending:
                event = pending.pop(0)
            else:
                try:
                    event = self._events.get(timeout=0.25)
                except queue.Empty:
                    continue
                if event is None:
                    continue
            if event.get("_request") == "last_location":
                self._fetch_last_location()
                continue
            if not self._deliver(event):
                self._spool(event)
                pending = self._read_spool() + pending

    def stop(self) -> None:
        if not self.enabled:
            return
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
