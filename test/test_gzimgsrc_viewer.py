import importlib.util
import unittest
from pathlib import Path


VIEWER = Path(__file__).parents[1] / "scripts/tools/view_camera.py"
spec = importlib.util.spec_from_file_location("view_camera", VIEWER)
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)


class GzImageViewerTest(unittest.TestCase):
    def test_builds_low_latency_bgr_appsink_pipeline(self) -> None:
        self.assertEqual(
            viewer.build_pipeline("/X3/front_camera/image"),
            "gzimgsrc topic=/X3/front_camera/image ! videoconvert "
            "! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false",
        )

    def test_reports_fps_over_latest_second(self) -> None:
        fps = viewer.RollingFps()

        self.assertEqual(fps.update(10.0), 0.0)
        self.assertAlmostEqual(fps.update(10.5), 2.0)
        self.assertAlmostEqual(fps.update(11.0), 2.0)
        self.assertAlmostEqual(fps.update(11.5), 2.0)


if __name__ == "__main__":
    unittest.main()
