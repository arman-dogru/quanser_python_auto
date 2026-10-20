import unittest

import numpy as np

from camera_freshness import CameraFreshness


class CameraFreshnessTests(unittest.TestCase):
    def test_repeated_raw_image_is_rejected_until_image_changes(self):
        guard = CameraFreshness()
        first = np.zeros((480, 640), dtype=np.uint8)
        self.assertTrue(guard.check_image(first)[0])
        self.assertFalse(guard.check_image(first.copy())[0])
        self.assertEqual(guard.raw_duplicate_count, 1)
        changed = first.copy()
        changed[10, 10] = 1
        self.assertTrue(guard.check_image(changed)[0])
        self.assertEqual(guard.raw_duplicate_count, 0)

    def test_identical_detection_is_rejected_after_five_images(self):
        guard = CameraFreshness(max_same_detection=5)
        results = [guard.check_detection(True, 232.81, 234.52, 0.9539)
                   for _ in range(88)]
        self.assertEqual(results[:4], [True] * 4)
        self.assertFalse(any(results[4:]))
        self.assertTrue(guard.check_detection(True, 233.11, 234.52, 0.9539))

    def test_no_pad_resets_detector_streak(self):
        guard = CameraFreshness(max_same_detection=3)
        self.assertTrue(guard.check_detection(True, 1, 2, 0.8))
        self.assertTrue(guard.check_detection(True, 1, 2, 0.8))
        self.assertTrue(guard.check_detection(False, float('nan'), float('nan'), float('nan')))
        self.assertTrue(guard.check_detection(True, 1, 2, 0.8))

    def test_reconnect_does_not_reset_detection_guard(self):
        guard = CameraFreshness(max_same_detection=3)
        guard.check_detection(True, 1, 2, 0.8)
        guard.check_detection(True, 1, 2, 0.8)
        guard.reset_camera_connection()
        self.assertFalse(guard.check_detection(True, 1, 2, 0.8))


if __name__ == '__main__':
    unittest.main()
