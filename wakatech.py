#!/usr/bin/env python3
"""Production-oriented Wakatech Raspberry Pi/HiveMQ reference client.

Covers the MQTT contract in RASPBERRY_PI_MQTT.md. Hardware teams must replace
example sensor values and implement/test physical detection, feedback, power and
OTA behavior. The backend still lacks durable fall acknowledgements, so falls
remain queued after MQTT PUBACK and are retried conservatively.

Dependency: paho-mqtt>=2,<3
"""

from __future__ import annotations

import argparse, hashlib, hmac, json, logging, os, random, signal, sqlite3, ssl
import stat, subprocess, threading, time, uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import paho.mqtt.client as mqtt

MAX_PAYLOAD = 16_384
PRIORITY = {
    "fall.opened": 1,
    "settings.applied": 2,
    "device.heartbeat": 3,
    "walk-session.completed": 4,
    "activity.checkpoint": 5,
}
SUFFIX = {
    "fall.opened": "events/fall",
    "settings.applied": "events/settings-applied",
    "device.heartbeat": "telemetry/heartbeat",
    "walk-session.completed": "events/walk-session",
    "activity.checkpoint": "events/activity",
}


# --- Hardcoded device credentials -------------------------------------------
HOST        = "e2734df303a542cba4690f4a824a98a9.s1.eu.hivemq.cloud"
PORT        = 8883
TLS         = True
USERNAME    = "device-WK-2026-000042"
PASSWORD    = "12312345678"
HARDWARE_ID = "WK-2026-000042"
SECRET      = "b7keQePuai1a-Qf5jNpqW9VNLhueQDK-4BM3l4M1y_HK3Rnnu3OES_Hv3h4K9Ys-"
CA_FILE     = None
FIRMWARE    = "1.0.0"
PROTOCOL    = 1
DB_PATH     = Path(__file__).parent / "wakatech_state.db"
MAX_ROWS    = 5000
if PROTOCOL != 1:
    raise RuntimeError("Only protocol version 1 is supported")
if not 1 <= len(FIRMWARE) <= 40:
    raise RuntimeError("Invalid firmware version")
if PORT == 8883 and not TLS:
    raise RuntimeError("TLS is required on port 8883")

ROOT = f"wakatech/v1/devices/{HARDWARE_ID}"
SETTINGS_TOPIC = f"{ROOT}/commands/settings"
CLIENT_ID = f"wakatech-stick-{HARDWARE_ID}"
KEY = hashlib.sha256(SECRET.encode()).hexdigest()
BOOT_ID, STOP, CONNECTED = str(uuid.uuid4()), threading.Event(), threading.Event()


def now() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def compact(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=False)


def parse_time(value: Any) -> datetime:
    if not isinstance(value, str):
        raise ValueError("Timestamp must be a string")
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


class Store:
    """Power-loss-resistant local settings and bounded priority outbox."""

    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        try:
            path.chmod(stat.S_IRUSR | stat.S_IWUSR)
        except OSError:
            logging.warning("Could not restrict permissions on %s", path)
        self.db.row_factory = sqlite3.Row
        with self.db:
            self.db.execute("PRAGMA journal_mode=WAL")
            self.db.execute("PRAGMA synchronous=FULL")
            self.db.executescript("""
              CREATE TABLE IF NOT EXISTS outbox(
                id INTEGER PRIMARY KEY, message_id TEXT UNIQUE NOT NULL,
                event_type TEXT NOT NULL, topic TEXT NOT NULL,
                envelope_json TEXT NOT NULL, priority INTEGER NOT NULL,
                attempts INTEGER NOT NULL DEFAULT 0, next_attempt REAL NOT NULL DEFAULT 0,
                broker_acked INTEGER NOT NULL DEFAULT 0, created_at REAL NOT NULL);
              CREATE INDEX IF NOT EXISTS outbox_due ON outbox(next_attempt,priority,created_at);
              CREATE TABLE IF NOT EXISTS settings(
                singleton INTEGER PRIMARY KEY CHECK(singleton=1), version INTEGER NOT NULL,
                sensitivity TEXT NOT NULL, grace_seconds INTEGER NOT NULL, updated_at TEXT NOT NULL);
              CREATE TABLE IF NOT EXISTS runtime_state(
                key TEXT PRIMARY KEY, value INTEGER NOT NULL);
            """)
            self.db.execute(
                "INSERT OR IGNORE INTO settings VALUES(1,0,'medium',0,?)", (now(),)
            )
            self.db.execute("INSERT OR IGNORE INTO runtime_state VALUES('sequence',0)")

    def enqueue(self, event_type: str, data: dict[str, Any]) -> str:
        if event_type not in PRIORITY:
            raise ValueError("Unsupported event type")
        message_id = str(uuid.uuid4())
        with self.lock, self.db:
            self.db.execute(
                "UPDATE runtime_state SET value=value+1 WHERE key='sequence'"
            )
            sequence = self.db.execute(
                "SELECT value FROM runtime_state WHERE key='sequence'"
            ).fetchone()["value"]
            envelope = {
                "schemaVersion": 1,
                "messageId": message_id,
                "deviceId": HARDWARE_ID,
                "eventType": event_type,
                "occurredAt": now(),
                "sequence": sequence,
                "bootId": BOOT_ID,
                "firmwareVersion": FIRMWARE,
                "data": data,
            }
            body = compact(envelope)
            if len(body.encode()) >= MAX_PAYLOAD:
                raise ValueError("Envelope is too large")
            # Coalesce obsolete health/cumulative data; never coalesce safety records.
            if event_type in {"device.heartbeat", "activity.checkpoint"}:
                self.db.execute("DELETE FROM outbox WHERE event_type=?", (event_type,))
            count = self.db.execute("SELECT COUNT(*) n FROM outbox").fetchone()["n"]
            if count >= MAX_ROWS:
                self.db.execute(
                    "DELETE FROM outbox WHERE id=(SELECT id FROM outbox WHERE event_type "
                    "IN('device.heartbeat','activity.checkpoint') ORDER BY created_at LIMIT 1)"
                )
                count = self.db.execute("SELECT COUNT(*) n FROM outbox").fetchone()["n"]
                if count >= MAX_ROWS and event_type not in {
                    "fall.opened",
                    "settings.applied",
                }:
                    raise RuntimeError("Outbox full; safety records preserved")
            self.db.execute(
                "INSERT INTO outbox(message_id,event_type,topic,envelope_json,priority,created_at) "
                "VALUES(?,?,?,?,?,?)",
                (
                    message_id,
                    event_type,
                    f"{ROOT}/{SUFFIX[event_type]}",
                    body,
                    PRIORITY[event_type],
                    time.time(),
                ),
            )
        return message_id

    def due(self):
        with self.lock:
            return self.db.execute(
                "SELECT * FROM outbox WHERE next_attempt<=? "
                "ORDER BY priority,created_at LIMIT 1",
                (time.time(),),
            ).fetchone()

    def success(self, row):
        with self.lock, self.db:
            if row["event_type"] == "fall.opened":
                # MQTT PUBACK is not proof of durable backend storage.
                self.db.execute(
                    "UPDATE outbox SET broker_acked=1,attempts=attempts+1,next_attempt=? WHERE id=?",
                    (time.time() + 300 + random.uniform(0, 30), row["id"]),
                )
            else:
                self.db.execute("DELETE FROM outbox WHERE id=?", (row["id"],))

    def failure(self, row):
        delay = min(300, 2 ** min(row["attempts"], 8)) + random.uniform(0, 1)
        with self.lock, self.db:
            self.db.execute(
                "UPDATE outbox SET attempts=attempts+1,next_attempt=? WHERE id=?",
                (time.time() + delay, row["id"]),
            )

    def settings(self):
        with self.lock:
            return self.db.execute(
                "SELECT * FROM settings WHERE singleton=1"
            ).fetchone()

    def apply(self, version: int, sensitivity: str, grace: int):
        with self.lock, self.db:
            self.db.execute(
                "UPDATE settings SET version=?,sensitivity=?,grace_seconds=?,updated_at=? WHERE singleton=1",
                (version, sensitivity, grace, now()),
            )

    def counts(self):
        with self.lock:
            return {
                r["event_type"]: r["n"]
                for r in self.db.execute(
                    "SELECT event_type,COUNT(*) n FROM outbox GROUP BY event_type"
                )
            }

    def close(self):
        with self.lock:
            self.db.close()


STORE = Store(DB_PATH)


def signed(envelope_json: str) -> bytes:
    timestamp = now()  # Fresh auth timestamp; original envelope/IDs remain unchanged.
    signature = hmac.new(
        KEY.encode(), f"{timestamp}.{envelope_json}".encode(), hashlib.sha256
    ).hexdigest()
    payload = (
        '{"envelope":'
        + envelope_json
        + ',"auth":{"timestamp":"'
        + timestamp
        + '","signature":"'
        + signature
        + '"}}'
    ).encode()
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("MQTT payload exceeds 16,384 bytes")
    return payload


def validate_location(location: dict[str, Any]):
    if not -90 <= float(location["latitude"]) <= 90:
        raise ValueError("Invalid latitude")
    if not -180 <= float(location["longitude"]) <= 180:
        raise ValueError("Invalid longitude")
    if float(location.get("accuracyMeters", 0)) < 0:
        raise ValueError("Invalid accuracy")
    parse_time(location["fixAt"])


def queue_heartbeat(battery: int, rssi: int, location=None):
    if not 0 <= battery <= 100:
        raise ValueError("Invalid battery")
    data = {"batteryPercent": battery, "wifiRssi": rssi, "protocolVersion": PROTOCOL}
    if location is not None:
        validate_location(location)
        data["location"] = location
    return STORE.enqueue("device.heartbeat", data)


def queue_fall(severity: str, battery: int, location: dict, fall_id=None):
    """Invoke only after physical false-alarm grace/cancellation completes."""
    if severity not in {"high", "critical"}:
        raise ValueError("Invalid severity")
    if not 0 <= battery <= 100:
        raise ValueError("Invalid battery")
    validate_location(location)
    return STORE.enqueue(
        "fall.opened",
        {
            "fallEventId": fall_id or str(uuid.uuid4()),
            "severity": severity,
            "batteryPercent": battery,
            "location": location,
        },
    )


def queue_activity(steps: int, epoch: str, sample_id=None):
    if steps < 0:
        raise ValueError("Invalid cumulative steps")
    return STORE.enqueue(
        "activity.checkpoint",
        {
            "sampleId": sample_id or str(uuid.uuid4()),
            "counterEpoch": epoch,
            "cumulativeSteps": steps,
        },
    )


def queue_walk(start: str, end: str, steps: int):
    if parse_time(end) < parse_time(start) or steps < 0:
        raise ValueError("Invalid walking session")
    return STORE.enqueue(
        "walk-session.completed",
        {
            "sessionId": str(uuid.uuid4()),
            "startedAt": start,
            "endedAt": end,
            "steps": steps,
        },
    )


def settings_ack(version: int, status: str, error=None):
    data = {"version": version, "status": status}
    if error:
        data["errorCode"] = error
    return STORE.enqueue("settings.applied", data)


def verify_command(wrapper: dict) -> dict:
    command, auth = wrapper.get("command"), wrapper.get("auth")
    if not isinstance(command, dict) or not isinstance(auth, dict):
        raise ValueError("Malformed wrapper")
    supplied = auth.get("signature")
    expected = hmac.new(
        KEY.encode(), compact(command).encode(), hashlib.sha256
    ).hexdigest()
    if not isinstance(supplied, str) or not hmac.compare_digest(supplied, expected):
        raise ValueError("Invalid command signature")
    if command.get("schemaVersion") != 1 or command.get("deviceId") != HARDWARE_ID:
        raise ValueError("Invalid command identity/schema")
    if command.get("commandType") != "settings.update":
        raise ValueError("Unsupported command")
    if not isinstance(command.get("commandId"), str):
        raise ValueError("Missing command ID")
    parse_time(command.get("issuedAt"))
    if not isinstance(command.get("version"), int) or command["version"] <= 0:
        raise ValueError("Invalid version")
    return command


def handle_settings(payload: bytes):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("Settings exceed 16 KiB")
    command = verify_command(json.loads(payload.decode()))
    version, current = command["version"], STORE.settings()
    if version < current["version"]:
        logging.info("Ignored stale settings v%s", version)
        return
    if version == current["version"]:
        settings_ack(version, "applied")
        return
    data = command.get("data")
    if not isinstance(data, dict):
        settings_ack(version, "rejected", "INVALID_SETTINGS")
        return
    sensitivity, grace = data.get("fallSensitivity"), data.get("falseAlarmGraceSeconds")
    if sensitivity not in {"low", "medium", "high"}:
        settings_ack(version, "rejected", "UNSUPPORTED_SENSITIVITY")
        return
    if not isinstance(grace, int) or not 0 <= grace <= 120:
        settings_ack(version, "rejected", "INVALID_GRACE_SECONDS")
        return
    STORE.apply(version, sensitivity, grace)  # Atomic persistence before ACK.
    settings_ack(version, "applied")
    logging.info("Applied settings v%s", version)


def on_connect(client, _u, _f, reason, _p):
    if reason != 0:
        logging.error("Connection rejected: %s", reason)
        return
    rc, _ = client.subscribe(SETTINGS_TOPIC, qos=1)
    if rc == mqtt.MQTT_ERR_SUCCESS:
        CONNECTED.set()
        logging.info("Subscribed to %s", SETTINGS_TOPIC)
    else:
        logging.error("Settings subscription failed: %s", rc)


def on_disconnect(_c, _u, _f, reason, _p):
    CONNECTED.clear()
    if not STOP.is_set():
        logging.warning("Disconnected: %s", reason)


def on_message(_c, _u, message):
    if message.topic != SETTINGS_TOPIC:
        return
    try:
        handle_settings(message.payload)
    except Exception as error:
        logging.error("Rejected settings command: %s", error)


def publish_worker(client):
    while not STOP.is_set():
        row = STORE.due()
        if row is None or not CONNECTED.wait(1):
            STOP.wait(0.25)
            continue
        try:
            info = client.publish(
                row["topic"], signed(row["envelope_json"]), qos=1, retain=False
            )
            info.wait_for_publish(15)
            if not info.is_published():
                raise TimeoutError("No MQTT PUBACK")
            STORE.success(row)
            logging.info("PUBACK %s %s", row["event_type"], row["message_id"])
        except Exception as error:
            STORE.failure(row)
            logging.warning("Publish deferred: %s", error)


def heartbeat_worker():
    while not STOP.is_set():
        if CONNECTED.is_set():
            try:
                queue_heartbeat(78, -61)  # Replace with actual validated readings.
            except Exception as error:
                logging.error("Heartbeat queue failed: %s", error)
        STOP.wait(60)  # Current heartbeat only; obsolete ones are coalesced.


def make_client():
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id=CLIENT_ID,
        clean_session=False,
        protocol=mqtt.MQTTv311,
    )
    client.username_pw_set(USERNAME, PASSWORD)
    if TLS:
        context = ssl.create_default_context(cafile=CA_FILE)
        context.check_hostname, context.verify_mode = True, ssl.CERT_REQUIRED
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        client.tls_set_context(context)
    client.on_connect, client.on_disconnect, client.on_message = (
        on_connect,
        on_disconnect,
        on_message,
    )
    return client


def run(enable_sample_heartbeat: bool = True):
    """Run the MQTT network loop.

    ``enable_sample_heartbeat`` exists for the standalone reference program.
    The integrated walking-stick application disables it and queues heartbeats
    from its real runtime state instead.
    """
    client, delay = make_client(), 1
    workers = [
        threading.Thread(
            target=publish_worker, args=(client,), name="mqtt-publisher", daemon=True
        )
    ]
    if enable_sample_heartbeat:
        workers.append(
            threading.Thread(
                target=heartbeat_worker, name="mqtt-heartbeat", daemon=True
            )
        )
    for worker in workers:
        worker.start()
    while not STOP.is_set():
        try:
            client.connect(HOST, PORT, keepalive=60)
            delay = 1
            while not STOP.is_set() and client.loop(timeout=1) == mqtt.MQTT_ERR_SUCCESS:
                pass
            if not STOP.is_set():
                raise ConnectionError("MQTT network loop stopped")
        # Paho 2.x does not consistently export MQTTException from
        # paho.mqtt.client. Network/TLS failures and callback-library errors
        # must not terminate the long-running device thread.
        except Exception as error:
            CONNECTED.clear()
            wait = min(60, delay) + random.uniform(0, min(5, delay))
            logging.warning("Connect failed (%s), retry in %.1fs", error, wait)
            STOP.wait(wait)
            delay = min(60, delay * 2)
    try:
        client.disconnect()
        client.loop(timeout=1)
    except Exception:
        pass
    for worker in workers:
        worker.join(timeout=2.0)


def settings_snapshot() -> dict[str, Any]:
    """Return a plain copy safe for use by the integrated application."""
    row = STORE.settings()
    return {
        "version": row["version"],
        "fallSensitivity": row["sensitivity"],
        "falseAlarmGraceSeconds": row["grace_seconds"],
        "updatedAt": row["updated_at"],
    }


def request_stop() -> None:
    """Ask an embedded MQTT loop and its workers to stop cleanly."""
    STOP.set()


def ntp_status() -> bool | None:
    """Report systemd-timesyncd state without making clock changes."""
    try:
        result = subprocess.run(
            ["timedatectl", "show", "--property=NTPSynchronized", "--value"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        value = result.stdout.strip().lower()
        return value == "yes" if value in {"yes", "no"} else None
    except (FileNotFoundError, subprocess.SubprocessError):
        return None


def state_file_secure() -> bool | None:
    """The service should create its state with no group/world permissions."""
    try:
        return not bool(DB_PATH.stat().st_mode & (stat.S_IRWXG | stat.S_IRWXO))
    except OSError:
        return None


def diagnostics():
    s = STORE.settings()
    print(
        compact(
            {
                "hardwareId": HARDWARE_ID,
                "clientId": CLIENT_ID,
                "mqttHost": HOST,
                "mqttPort": PORT,
                "tls": TLS,
                "database": str(DB_PATH),
                "queued": STORE.counts(),
                "settingsVersion": s["version"],
                "firmwareVersion": FIRMWARE,
                "protocolVersion": PROTOCOL,
                "ntpSynchronized": ntp_status(),
                "stateFilePermissionsSecure": state_file_secure(),
                "secretsPresent": bool(PASSWORD and SECRET),
            }
        )
    )


def location():
    return {
        # Development fallback: FUT Minna main campus, Gidan Kwano.
        "latitude": 9.53101,
        "longitude": 6.4469,
        "accuracyMeters": 12.4,
        "fixAt": now(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command",
        nargs="?",
        default="run",
        choices=["run", "diagnostics", "heartbeat", "activity", "fall", "walk"],
    )
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    signal.signal(signal.SIGTERM, lambda *_: STOP.set())
    signal.signal(signal.SIGINT, lambda *_: STOP.set())
    try:
        if args.command == "run":
            run()
        elif args.command == "diagnostics":
            diagnostics()
        elif args.command == "heartbeat":
            print(queue_heartbeat(78, -61, location()))
        elif args.command == "activity":
            print(queue_activity(1684, "example-counter-epoch"))
        elif args.command == "fall":
            print(queue_fall("critical", 77, location()))
        elif args.command == "walk":
            start = (
                datetime.fromtimestamp(time.time() - 1800, timezone.utc)
                .isoformat(timespec="milliseconds")
                .replace("+00:00", "Z")
            )
            print(queue_walk(start, now(), 1250))
    finally:
        STOP.set()
        STORE.close()


if __name__ == "__main__":
    main()
