"""
test_camera.py — Standalone Camera + YOLO Test

Tests the Picamera2 CSI camera with YOLO ONNX inference.
Use this to verify the camera and model work before running main.py.

Usage
─────
    python test_camera.py
    python test_camera.py --model yolo11n.onnx --conf 0.5
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path
from queue import Queue

import cv2

from audio import SpeechManager
from export_model import export_pt_to_onnx

_running = True

def _signal_handler(sig, frame):
    global _running
    _running = False

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


class FastPiCamera:
    """Threaded Picamera2 wrapper for low-latency BGR frame capture."""

    def __init__(self, width: int = 640, height: int = 480):
        from picamera2 import Picamera2

        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(
            main={"size": (width, height), "format": "BGR888"}
        )
        self.camera.configure(config)
        self.frame_queue: Queue = Queue(maxsize=2)
        self.running = False

    def start(self):
        self.camera.start()
        time.sleep(1.0)
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        return self

    def _update(self):
        while self.running:
            if not self.frame_queue.full():
                frame = self.camera.capture_array()
                self.frame_queue.put(frame)

    def get_frame(self):
        if not self.frame_queue.empty():
            return self.frame_queue.get()
        return None

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)
        self.camera.stop()


def _has_display() -> bool:
    return bool(os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY") or os.getenv("QT_QPA_PLATFORM"))


def _prepare_model(model: str) -> str:
    path = Path(model)
    if path.exists():
        if path.suffix.lower() == ".pt":
            return str(export_pt_to_onnx(path, output_dir=Path(__file__).resolve().parent))
        return str(path)

    normalized = model.lower()
    if normalized.startswith("yolo11") or normalized.startswith("yolov11"):
        local_onnx = Path(__file__).resolve().parent / f"{normalized}.onnx"
        if local_onnx.exists():
            return str(local_onnx)
        # Do not attempt PT->ONNX conversion on-device. Require a pre-exported ONNX file.
        sys.exit(
            f"[ERROR] ONNX model {local_onnx.name} not found."
            f" Copy the pre-exported ONNX into {Path(__file__).resolve().parent}"
            f" or export on a desktop and transfer it to the Pi."
        )

    return model


def main():
    parser = argparse.ArgumentParser(description="Camera + YOLO Test")
    parser.add_argument("--model", type=str, default="yolo11n",
                        help="YOLO model alias or path (default: yolo11n)")
    parser.add_argument("--conf", type=float, default=0.45,
                        help="Confidence threshold (default: 0.45)")
    parser.add_argument("--headless", action="store_true",
                        help="Run without opening a display window")
    parser.add_argument("--no-tts", action="store_true",
                        help="Disable spoken object announcements")
    parser.add_argument("--tts-cooldown", type=float, default=8.0,
                        help="Seconds before repeating an object's announcement")
    parser.add_argument("--audio-device", default=os.getenv("WALKING_STICK_AUDIO_DEVICE"),
                        help="Optional PipeWire sink ID or ALSA device; otherwise detect automatically")
    args = parser.parse_args()

    headless = args.headless or not _has_display()
    if headless and not args.headless:
        print("[WARN] No display detected. Running in headless mode and skipping GUI window.")

    print("=" * 56)
    print("  Camera + YOLO Test")
    print("=" * 56)
    print()

    from ultralytics import YOLO
    model_source = _prepare_model(args.model)
    model = YOLO(model_source, task="detect")
    print(f"[INFO] Model loaded: {model_source}")

    speech = SpeechManager(
        enabled=not args.no_tts,
        cooldown=args.tts_cooldown,
        device=args.audio_device,
    ).start()
    print("[INFO] Starting camera...")
    camera = FastPiCamera().start()
    print("[INFO] Press 'q' or Ctrl-C to stop.\n")

    try:
        while _running:
            frame = camera.get_frame()
            if frame is None:
                continue

            results = model.predict(frame, conf=args.conf, verbose=False)
            annotated_frame = results[0].plot()

            if results[0].boxes is not None and len(results[0].boxes) > 0:
                object_names = {results[0].names[int(box.cls[0])] for box in results[0].boxes}
                for object_name in sorted(object_names):
                    print(f"Object detected: {object_name}.")
                    speech.announce(
                        f"{object_name} detected.",
                        key=f"vision:{object_name.lower()}",
                    )

            if not headless:
                cv2.imshow("Walking Stick Vision", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                # Headless mode: keep the loop alive without GUI.
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        speech.stop()
        camera.stop()
        if not headless:
            cv2.destroyAllWindows()
        print("[INFO] Camera released. Exiting.")


if __name__ == "__main__":
    main()
