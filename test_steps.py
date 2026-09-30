import unittest

from steps import StepCounter


class StepCounterTests(unittest.TestCase):
    def test_counts_separated_peaks_only(self):
        counter = StepCounter()
        self.assertTrue(counter.update(1.3, 0.0))
        self.assertFalse(counter.update(1.4, 0.1))
        self.assertFalse(counter.update(1.0, 0.2))
        self.assertTrue(counter.update(1.3, 0.5))
        self.assertEqual(counter.count, 2)


if __name__ == "__main__":
    unittest.main()