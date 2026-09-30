"""Reliable, non-blocking text-to-speech for the walking stick."""

from __future__ import annotations

import queue
import os
import re
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class SpeechMessage:
    text: str
    key: str
    priority: int


class SystemSpeechEngine:
    """Generate speech with espeak-ng and play it through available sinks."""

    def __init__(self, device: str | None = None) -> None:
        self.rate = 165
        self.volume = 1.0
        self._players = self._find_players(device or os.getenv("WALKING_STICK_AUDIO_DEVICE"))
        if not self._players:
            raise RuntimeError("no usable audio playback device found")
        print(f"[INFO] Audio output candidates: {', '.join(name for name, _ in self._players)}")

    @staticmethod
    def _find_players(device: str | None) -> list[tuple[str, list[str]]]:
        players: list[tuple[str, list[str]]] = []
        if not shutil.which("espeak-ng"):
            return players

        if device:
            if shutil.which("pw-play"):
                players.append((f"PipeWire {device}", ["pw-play", "--target", device, "-"]))
            if shutil.which("aplay"):
                players.append((f"ALSA {device}", ["aplay", "-q", "-D", device]))
            return players

        # PipeWire owns both desktop jack and Bluetooth routing when available.
        if shutil.which("pw-play") and shutil.which("wpctl"):
            try:
                status = subprocess.run(
                    ["wpctl", "status", "-n"], capture_output=True, text=True,
                    check=False, timeout=2,
                ).stdout
                in_sinks = False
                for line in status.splitlines():
                    if "Sinks:" in line:
                        in_sinks = True
                        continue
                    if in_sinks and "Sources:" in line:
                        break
                    if in_sinks and (match := re.search(r"(?:\*\s*)?(\d+)\.", line)):
                        sink_id = match.group(1)
                        players.append((f"PipeWire sink {sink_id}", ["pw-play", "--target", sink_id, "-"]))
            except (OSError, subprocess.SubprocessError):
                pass

        if shutil.which("aplay"):
            players.append(("ALSA default", ["aplay", "-q", "-D", "default"]))
            try:
                devices = subprocess.run(
                    ["aplay", "-L"], capture_output=True, text=True,
                    check=False, timeout=2,
                ).stdout.splitlines()
                for name in devices:
                    if name and not name.startswith(" ") and any(
                        marker in name.lower() for marker in ("bluetooth", "bluez", "headphones")
                    ):
                        players.append((f"ALSA {name}", ["aplay", "-q", "-D", name]))
            except (OSError, subprocess.SubprocessError):
                pass
        return players

    def setProperty(self, name: str, value: object) -> None:
        if name == "rate":
            self.rate = int(value)
        elif name == "volume":
            self.volume = float(value)

    def say(self, text: str) -> None:
        speech = subprocess.run(
            ["espeak-ng", "--stdout", "-s", str(self.rate),
             "-a", str(max(0, min(200, int(self.volume * 100)))), text],
            capture_output=True, check=True, timeout=15,
        ).stdout
        if not speech:
            raise RuntimeError("speech synthesizer returned no audio data")
        last_error: Exception | None = None
        for name, command in self._players:
            try:
                subprocess.run(command, input=speech, capture_output=True, check=True, timeout=15)
                return
            except (OSError, subprocess.SubprocessError) as exc:
                last_error = exc
                print(f"[WARN] Audio output {name} failed: {exc}")
        raise RuntimeError(f"all audio outputs failed: {last_error}")

    def runAndWait(self) -> None:
        return


class SpeechManager:
    """Speak events on a worker thread without blocking sensors or vision."""

    def __init__(
        self,
        enabled: bool = True,
        rate: int = 165,
        volume: float = 1.0,
        cooldown: float = 5.0,
        engine_factory: Callable[[], object] | None = None,
        device: str | None = None,
    ) -> None:
        self.enabled = enabled
        self.rate = max(80, min(rate, 300))
        self.volume = max(0.0, min(volume, 1.0))
        self.cooldown = max(0.0, cooldown)
        self._engine_factory = engine_factory or (lambda: self._create_engine(device))
        self._messages: queue.PriorityQueue[tuple[int, int, SpeechMessage]] = queue.PriorityQueue(maxsize=32)
        self._last_spoken: dict[str, float] = {}
        self._sequence = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    @staticmethod
    def _create_engine(device: str | None = None) -> object:
        return SystemSpeechEngine(device=device)

    def start(self) -> "SpeechManager":
        if not self.enabled or self._running:
            return self
        self._running = True
        self._thread = threading.Thread(target=self._run, name="tts", daemon=True)
        self._thread.start()
        return self

    def announce(
        self,
        text: str,
        key: str | None = None,
        cooldown: float | None = None,
        priority: int = 10,
    ) -> bool:
        """Queue an announcement unless the same event is cooling down."""
        if not self.enabled or not text.strip():
            return False

        event_key = key or text
        now = time.monotonic()
        event_cooldown = self.cooldown if cooldown is None else max(0.0, cooldown)
        with self._lock:
            if now - self._last_spoken.get(event_key, -float("inf")) < event_cooldown:
                return False
            self._last_spoken[event_key] = now
            self._sequence += 1
            message = (priority, self._sequence, SpeechMessage(text.strip(), event_key, priority))

        try:
            self._messages.put_nowait(message)
            return True
        except queue.Full:
            # Audio must never become a back-pressure source for the main loop.
            with self._lock:
                self._last_spoken.pop(event_key, None)
            return False

    def _run(self) -> None:
        try:
            engine = self._engine_factory()
            engine.setProperty("rate", self.rate)
            engine.setProperty("volume", self.volume)
        except Exception as exc:
            print(f"[WARN] Text-to-speech unavailable: {exc}")
            self._running = False
            return

        while self._running:
            try:
                _, _, message = self._messages.get(timeout=0.25)
            except queue.Empty:
                continue
            try:
                engine.say(message.text)
                engine.runAndWait()
            except Exception as exc:
                print(f"[WARN] Text-to-speech failed: {exc}")

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
