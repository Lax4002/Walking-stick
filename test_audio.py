"""Run the camera object speech test and test the speech queue."""

import sys
import threading
import unittest

from audio import SpeechManager


class FakeEngine:
    def __init__(self):
        self.spoken = []
        self.finished = threading.Event()

    def setProperty(self, name, value):
        pass

    def say(self, text):
        self.spoken.append(text)

    def runAndWait(self):
        self.finished.set()


class SpeechManagerTests(unittest.TestCase):
    def test_speaks_in_background_and_suppresses_duplicate_event(self):
        engine = FakeEngine()
        speech = SpeechManager(engine_factory=lambda: engine, cooldown=10).start()
        try:
            self.assertTrue(speech.announce("Fall detected", key="fall"))
            self.assertFalse(speech.announce("Fall detected", key="fall"))
            self.assertTrue(engine.finished.wait(timeout=1.0))
            self.assertEqual(engine.spoken, ["Fall detected"])
        finally:
            speech.stop()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in {"-h", "--help"}:
        print("Usage: python test_audio.py [camera test options]")
        print("  --model PATH       YOLO model alias or path")
        print("  --conf FLOAT       Detection confidence threshold")
        print("  --headless         Run without a camera window")
        print("  --no-tts           Disable spoken announcements")
        print("  --tts-cooldown SEC Repeat delay for each object")
        print("  --audio-device ID  Optional PipeWire sink ID or ALSA device")
        raise SystemExit(0)

    from test_camera import main as camera_test

    camera_test()