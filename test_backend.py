import tempfile
import unittest
from pathlib import Path

from backend import BackendClient


class BackendClientTests(unittest.TestCase):
    def test_publish_builds_stable_event_envelope(self):
        with tempfile.TemporaryDirectory() as directory:
            client = BackendClient("https://example.test/events", "stick-01", Path(directory) / "events.jsonl")
            self.assertTrue(client.publish("alert", {"alert_type": "fall"}, priority="critical"))
            event = client._events.get_nowait()
            self.assertEqual(event["device_id"], "stick-01")
            self.assertEqual(event["event_type"], "alert")
            self.assertEqual(event["priority"], "critical")
            self.assertEqual(event["payload"]["alert_type"], "fall")
            self.assertTrue(event["event_id"])
            self.assertTrue(event["occurred_at"].endswith("Z"))


if __name__ == "__main__":
    unittest.main()