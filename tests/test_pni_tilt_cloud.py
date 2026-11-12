import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pni_tilt_cloud_core import TiltCloud, rotate


def packet(tick, roll=0, pitch=0, heading=0, **extra):
    return {"tick": tick, "pni": {"valid": True, "status": 0,
        "update_tick": tick, "roll": roll, "pitch": pitch, "heading": heading, **extra}}


def settled(core, pitch=0, roll=0, heading=0):
    for i in range(11):
        core.ingest(packet(1000 + i * 100, roll, pitch, heading), 100 + i * .1)


def scan(core, ranges=(2,), start=100.8, now=101.0):
    return core.add_scan(ranges, 0, math.pi / 2, .1, 10, start, .1, now)


class TiltTests(unittest.TestCase):
    def test_rotation_preserves_axes_and_length(self):
        p = rotate((2, 3, 0), 0, math.pi / 2, 0)
        self.assertAlmostEqual(p[0], 0)
        self.assertAlmostEqual(p[1], 3)
        self.assertAlmostEqual(p[2], -2)
        self.assertAlmostEqual(sum(x*x for x in p), 13)
        p = rotate((0, 2, 0), math.pi / 2, 0, 0)
        self.assertAlmostEqual(p[2], 2)

    def test_horizontal_and_tilted_scans(self):
        core = TiltCloud()
        settled(core, pitch=30)
        self.assertEqual(scan(core), 1)
        x, y, z = next(iter(core.voxels.values()))
        self.assertAlmostEqual(x, math.sqrt(3))
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(z, -1)
        self.assertEqual(len(core.packed_points()), 12)
        self.assertAlmostEqual(struct.unpack("<fff", core.packed_points())[2], -1)

    def test_motion_and_jumps_are_not_accumulated(self):
        core = TiltCloud()
        for i in range(11):
            core.ingest(packet(1000 + i*100, pitch=i*2), 100+i*.1)
        self.assertEqual(scan(core), 0)
        self.assertIn("hold still", core.reason)
        for i in range(11, 21):
            core.ingest(packet(1000+i*100, pitch=20), 100+i*.1)
        self.assertEqual(scan(core, start=101.8, now=102), 1)

    def test_cached_values_never_fake_settling_history(self):
        core = TiltCloud()
        core.ingest(packet(1000), 100)
        for i in range(1, 10):
            core.ingest({"tick": 1000+i*10, "pni": packet(1000)["pni"]}, 100+i*.01)
        self.assertEqual(len(core.history), 1)
        self.assertEqual(scan(core), 0)

    def test_invalid_stale_and_bad_scan_timestamps(self):
        core = TiltCloud()
        settled(core)
        self.assertEqual(scan(core, start=100.8, now=102), 0)
        self.assertEqual(scan(core, start=0), 0)
        self.assertEqual(scan(core, start=101.2), 0)
        self.assertFalse(core.ingest(packet(2200, valid=False), 101.2))
        self.assertEqual(scan(core, start=101, now=101.2), 0)
        self.assertFalse(core.ingest(packet(2300, pitch=float("nan")), 101.3))

    def test_invalid_ranges_and_bounded_memory(self):
        core = TiltCloud(max_points=2)
        settled(core)
        self.assertEqual(scan(core, (float("nan"), float("inf"), -.1, 20)), 0)
        self.assertEqual(scan(core, (2, 2, 2, 2)), 4)
        self.assertEqual(len(core.voxels), 2)
        self.assertEqual(scan(core, (2, 2, 2, 2)), 4)
        self.assertEqual(len(core.voxels), 2)
        core.clear()
        self.assertEqual(core.voxels, {})
        self.assertIsNone(core.yaw_zero)

    def test_heading_is_optional_and_relative_not_fused(self):
        a, b = TiltCloud(), TiltCloud(use_heading=True)
        settled(a, heading=90)
        settled(b, heading=90)
        scan(a); scan(b)
        self.assertEqual(list(a.voxels.values()), list(b.voxels.values()))
        for i in range(11, 21):
            a.ingest(packet(1000+i*100, heading=100), 100+i*.1)
            b.ingest(packet(1000+i*100, heading=100), 100+i*.1)
        scan(a, start=101.8, now=102); scan(b, start=101.8, now=102)
        self.assertEqual(len(a.voxels), 1)
        self.assertEqual(len(b.voxels), 2)

    def test_restart_clears_cloud_but_tick_wrap_does_not(self):
        core = TiltCloud()
        settled(core)
        scan(core)
        core.ingest(packet(10), 101.1)
        self.assertFalse(core.voxels)
        self.assertEqual(len(core.history), 1)
        core = TiltCloud()
        core.ingest(packet(0xFFFFFFF0), 100)
        core.ingest(packet(0x20), 100.1)
        self.assertEqual(len(core.history), 2)

    def test_mount_and_sensor_height(self):
        core = TiltCloud(mount_rpy_deg=(0, 0, 90), sensor_height=.5)
        settled(core)
        scan(core)
        x, y, z = next(iter(core.voxels.values()))
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 2)
        self.assertAlmostEqual(z, .5)


class WrapperTests(unittest.TestCase):
    def wrapper(self):
        cloud = lambda: types.SimpleNamespace(header=types.SimpleNamespace())
        field = lambda **kw: types.SimpleNamespace(**kw)
        field.FLOAT32 = 7
        modules = {
            "rclpy": types.SimpleNamespace(), "rclpy.node": types.SimpleNamespace(Node=object),
            "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
            "sensor_msgs": types.SimpleNamespace(), "sensor_msgs.msg": types.SimpleNamespace(
                LaserScan=object, PointCloud2=cloud, PointField=field),
            "std_msgs": types.SimpleNamespace(), "std_msgs.msg": types.SimpleNamespace(
                String=lambda: types.SimpleNamespace()),
            "std_srvs": types.SimpleNamespace(), "std_srvs.srv": types.SimpleNamespace(Empty=object, Trigger=object),
        }
        spec = importlib.util.spec_from_file_location("tilt_under_test",
            Path(__file__).resolve().parents[1]/"pni_tilt_cloud_preview.py")
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        node = module.TiltCloudNode.__new__(module.TiltCloudNode)
        node.core = TiltCloud()
        node.last_status_publish = 0.0
        node.frame = "scan_reference"
        node.pending = []
        node.test_publish_period = module.publish_period
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: object()))
        return node

    def test_cloud_publish_rate_validation(self):
        node = self.wrapper()
        self.assertAlmostEqual(node.test_publish_period(10.0), 0.1)
        self.assertAlmostEqual(node.test_publish_period(2.0), 0.5)
        for value in (0, -1, 31, float("nan"), float("inf"), True, "10"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                node.test_publish_period(value)

    def test_pointcloud_binary_layout_and_empty_cloud(self):
        node = self.wrapper()
        settled(node.core); scan(node.core)
        msg = node.cloud_message()
        self.assertEqual((msg.width, msg.height, msg.point_step, msg.row_step), (1, 1, 12, 12))
        self.assertEqual([f.offset for f in msg.fields], [0, 4, 8])
        self.assertEqual(struct.unpack("<fff", msg.data), (2, 0, 0))
        self.assertEqual(msg.header.frame_id, "scan_reference")
        node.core.clear()
        self.assertEqual(node.cloud_message().width, 0)

    def test_save_creates_unique_files_without_overwrite_and_clear(self):
        node = self.wrapper()
        with tempfile.TemporaryDirectory() as directory:
            node.save_dir = Path(directory)
            response = types.SimpleNamespace()
            node.save(None, response)
            self.assertFalse(response.success)
            settled(node.core); scan(node.core)
            node.save(None, response)
            self.assertTrue(response.success)
            first = Path(response.message)
            content = first.read_text()
            self.assertIn("element vertex 1", content)
            self.assertTrue(content.endswith("2.000000 0.000000 0.000000\n"))
            node.save(None, response)
            self.assertNotEqual(first, Path(response.message))
            self.assertEqual(first.read_text(), content)
            node.publish = Mock()
            node.clear(None, response)
            self.assertFalse(node.core.voxels)
            node.publish.assert_called_once()


if __name__ == "__main__":
    unittest.main()
