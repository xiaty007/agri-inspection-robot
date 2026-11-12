import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pni_yaw_fusion_core import YawFusion, wrap, constrain_to_short_arc


def packet(heading, tick=1000, update=None, status=0, valid=True):
    return {"tick": tick, "pni": {"heading": heading, "status": status,
                                  "valid": valid, "update_tick": tick if update is None else update}}


class FusionTests(unittest.TestCase):
    def test_zero_alignment_and_matching_positive_turn(self):
        fusion = YawFusion()
        fusion.ingest_pni(packet(-90), 100)
        first = fusion.update_odom(math.radians(-20), 100, 100)
        self.assertTrue(first["aligned"])
        self.assertEqual(first["pni_status"], 0)
        self.assertAlmostEqual(first["offset_deg"], 70)
        fusion.ingest_pni(packet(-60, 1100), 100.1)
        second = fusion.update_odom(math.radians(10), 100.1, 100.1)
        self.assertAlmostEqual(math.degrees(second["fused_yaw"]), 10)
        self.assertAlmostEqual(second["innovation_deg"], 0)

    def test_wrap_crossing_does_not_make_360_degree_jump(self):
        fusion = YawFusion()
        fusion.ingest_pni(packet(179), 100)
        fusion.update_odom(math.radians(179), 100, 100)
        fusion.ingest_pni(packet(-179, 1100), 100.1)
        result = fusion.update_odom(math.radians(-179), 100.1, 100.1)
        self.assertAlmostEqual(math.degrees(result["fused_yaw"]), -179)
        self.assertAlmostEqual(result["innovation_deg"], 0)

    def test_each_pni_measurement_is_used_only_once(self):
        fusion = YawFusion()
        fusion.ingest_pni(packet(0), 100)
        fusion.update_odom(0, 100, 100)
        self.assertFalse(fusion.ingest_pni(packet(0, tick=1050, update=1000), 100.05))
        result = fusion.update_odom(0.01, 100.1, 100.1)
        self.assertFalse(result["pni_used"])
        self.assertAlmostEqual(result["fused_yaw"], 0.01)

    def test_compass_constraint_reduces_synthetic_rf2o_drift(self):
        fusion = YawFusion()
        for index in range(101):
            now = 100 + index / 10
            fusion.ingest_pni(packet(0, tick=1000 + index * 100), now)
            result = fusion.update_odom(math.radians(index / 10), now, now)
        self.assertLess(abs(math.degrees(result["fused_yaw"])), 3)
        self.assertAlmostEqual(math.degrees(result["rf2o_yaw"]), 10)

    def test_stale_and_poor_quality_samples_are_not_fused(self):
        fusion = YawFusion(min_status=2)
        self.assertFalse(fusion.ingest_pni(packet(0), 100))
        self.assertFalse(fusion.update_odom(0, 100, 100)["aligned"])
        self.assertTrue(fusion.ingest_pni(packet(0, tick=1100, status=3), 100.1))
        fusion.update_odom(0, 100.1, 100.1)
        result = fusion.update_odom(0.01, 100.5, 100.7)
        self.assertFalse(result["pni_fresh"])
        self.assertFalse(result["pni_used"])
        self.assertIsNone(result["pni_yaw"])

    def test_bad_timestamps_and_heading_are_rejected(self):
        fusion = YawFusion()
        self.assertFalse(fusion.ingest_pni(packet(0, tick=2000, update=1000), 100))
        self.assertFalse(fusion.ingest_pni(packet(float("nan")), 100))
        self.assertFalse(fusion.ingest_pni(packet(181), 100))
        for stamp, now in ((0, 100), (100, 101), (101, 100)):
            with self.assertRaises(ValueError):
                fusion.update_odom(0, stamp, now)

    def test_temporal_mismatch_waits_and_large_jump_is_rejected(self):
        fusion = YawFusion()
        fusion.ingest_pni(packet(0), 100)
        self.assertFalse(fusion.update_odom(0, 99.6, 100)["aligned"])
        self.assertTrue(fusion.update_odom(0, 100, 100)["aligned"])
        fusion.ingest_pni(packet(80, 1100), 100.1)
        result = fusion.update_odom(0, 100.1, 100.1)
        self.assertFalse(result["pni_used"])
        self.assertEqual(result["reason"], "PNI/RF2O disagreement rejected; RF2O fallback")
        self.assertAlmostEqual(result["fused_yaw"], 0)

    def test_mcu_reset_realigns_but_tick_wrap_does_not(self):
        fusion = YawFusion()
        fusion.ingest_pni(packet(10, tick=10000), 100)
        fusion.update_odom(0, 100, 100)
        fusion.ingest_pni(packet(-50, tick=50), 100.1)
        result = fusion.update_odom(0.02, 100.1, 100.1)
        self.assertAlmostEqual(result["fused_yaw"], 0.02)
        fusion = YawFusion()
        fusion.ingest_pni(packet(10, tick=0xFFFFFFF0), 100)
        fusion.update_odom(0, 100, 100)
        offset = fusion.offset
        fusion.ingest_pni(packet(11, tick=0x20), 100.1)
        fusion.update_odom(math.radians(1), 100.1, 100.1)
        self.assertEqual(fusion.offset, offset)

    def test_odom_restart_and_out_of_order_messages(self):
        fusion = YawFusion()
        fusion.ingest_pni(packet(10), 100)
        fusion.update_odom(0, 100, 100)
        self.assertIsNone(fusion.update_odom(0, 99.9, 100))
        result = fusion.update_odom(0.5, 101, 101)
        self.assertAlmostEqual(result["fused_yaw"], 0.5)
        self.assertFalse(result["aligned"])
        self.assertAlmostEqual(wrap(math.radians(359)), math.radians(-1))

    def assert_on_short_arc(self, result):
        first, second = result["rf2o_yaw"], result["pni_yaw"]
        span = wrap(second - first)
        correction = wrap(result["fused_yaw"] - first)
        self.assertGreaterEqual(correction, min(0, span) - 1e-10)
        self.assertLessEqual(correction, max(0, span) + 1e-10)

    def test_slow_divergence_cannot_drag_filter_beyond_current_inputs(self):
        fusion = YawFusion()
        for index in range(101):
            now = 100 + index / 10
            fusion.ingest_pni(packet(-index, tick=1000 + index * 100), now)
            result = fusion.update_odom(0, now, now)
            self.assert_on_short_arc(result)
            if index > 30:
                self.assertTrue(result["fallback_to_rf2o"])
                self.assertAlmostEqual(result["fused_yaw"], 0)
        # Returning PNI must be accepted, rather than locked out by history.
        fusion.ingest_pni(packet(-2, tick=11200), 110.2)
        result = fusion.update_odom(0, 110.2, 110.2)
        self.assertTrue(result["pni_used"])
        self.assert_on_short_arc(result)

    def test_logged_failure_pattern_recovers_without_zero_realign(self):
        fusion = YawFusion()
        fusion.ingest_pni(packet(0), 100)
        fusion.update_odom(0, 100, 100)
        offset = fusion.offset
        # Representative values from the last recorded trial, not ground truth.
        for index, (rf, pni) in enumerate(((56.198, -2.53), (51.697, -31.76),
                                         (52.602, -70.98), (52.443, -13.12)), 1):
            now = 100 + index / 10
            fusion.ingest_pni(packet(pni, tick=1000 + index * 100), now)
            result = fusion.update_odom(math.radians(rf), now, now)
            self.assertTrue(result["fallback_to_rf2o"])
            self.assertAlmostEqual(result["fused_yaw"], math.radians(rf))
        fusion.ingest_pni(packet(-2.62, tick=1500), 100.5)
        result = fusion.update_odom(math.radians(28.489), 100.5, 100.5)
        self.assertAlmostEqual(math.degrees(result["fused_yaw"]), 28.489)
        fusion.ingest_pni(packet(27, tick=1600), 100.6)
        result = fusion.update_odom(math.radians(28), 100.6, 100.6)
        self.assertTrue(result["pni_used"])
        self.assertEqual(fusion.offset, offset)
        self.assert_on_short_arc(result)

    def test_stale_invalid_and_unpaired_inputs_remove_old_correction(self):
        for condition in ("stale", "invalid", "unpaired"):
            with self.subTest(condition=condition):
                fusion = YawFusion()
                fusion.ingest_pni(packet(0), 100)
                fusion.update_odom(0, 100, 100)
                fusion.ingest_pni(packet(-20, tick=1100), 100.1)
                result = fusion.update_odom(0, 100.1, 100.1)
                self.assertLess(result["fused_yaw"], 0)
                if condition == "stale":
                    stamp, now = 100.5, 100.7
                elif condition == "invalid":
                    fusion.ingest_pni(packet(0, tick=1200, valid=False), 100.2)
                    stamp = now = 100.2
                else:
                    fusion.ingest_pni(packet(-20, tick=1200), 100.5)
                    stamp, now = 100.2, 100.5
                result = fusion.update_odom(0.02, stamp, now)
                self.assertTrue(result["fallback_to_rf2o"])
                self.assertAlmostEqual(result["fused_yaw"], 0.02)

    def test_reused_pni_still_bounds_recursive_prediction(self):
        fusion = YawFusion()
        fusion.ingest_pni(packet(0), 100)
        fusion.update_odom(0, 100, 100)
        fusion.ingest_pni(packet(-20, tick=1100), 100.1)
        fusion.update_odom(0, 100.1, 100.1)
        # RF2O moves across the retained PNI without a new PNI measurement.
        result = fusion.update_odom(math.radians(-25), 100.2, 100.2)
        self.assertFalse(result["pni_used"])
        self.assertTrue(result["arc_clamped"])
        self.assert_on_short_arc(result)

    def test_short_arc_projection_at_wrap_and_in_both_directions(self):
        for first in range(-180, 181, 30):
            for span in (-29, -2, 0, 2, 29):
                for estimate in range(-180, 181, 15):
                    a, b, e = map(math.radians, (first, first + span, estimate))
                    result = {"rf2o_yaw": wrap(a), "pni_yaw": wrap(b),
                              "fused_yaw": constrain_to_short_arc(e, a, b)}
                    self.assert_on_short_arc(result)
        fusion = YawFusion()
        fusion.ingest_pni(packet(179), 100)
        fusion.update_odom(math.radians(179), 100, 100)
        fusion.ingest_pni(packet(-175, tick=1100), 100.1)
        result = fusion.update_odom(math.radians(178), 100.1, 100.1)
        self.assertTrue(result["pni_used"])
        self.assert_on_short_arc(result)

    def test_ambiguous_disagreement_limit_is_not_allowed(self):
        with self.assertRaises(ValueError):
            YawFusion(max_innovation_deg=180)


class RosWrapperTests(unittest.TestCase):
    def load_wrapper(self):
        def message():
            return types.SimpleNamespace(
                header=None, pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(), orientation=types.SimpleNamespace()),
                scale=types.SimpleNamespace(), color=types.SimpleNamespace(),
                lifetime=types.SimpleNamespace())
        message.ARROW, message.ADD, message.DELETE = 0, 0, 2
        modules = {
            "rclpy": types.SimpleNamespace(), "rclpy.node": types.SimpleNamespace(Node=object),
            "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
            "nav_msgs": types.SimpleNamespace(), "nav_msgs.msg": types.SimpleNamespace(Odometry=object),
            "std_msgs": types.SimpleNamespace(), "std_msgs.msg": types.SimpleNamespace(String=lambda: types.SimpleNamespace()),
            "visualization_msgs": types.SimpleNamespace(),
            "visualization_msgs.msg": types.SimpleNamespace(Marker=message,
                MarkerArray=lambda: types.SimpleNamespace(markers=[])),
        }
        spec = importlib.util.spec_from_file_location("preview_under_test",
                    Path(__file__).resolve().parents[1] / "pni_yaw_fusion_preview.py")
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        return module

    def test_arrow_frame_colors_lifetime_and_stale_deletion(self):
        module = self.load_wrapper()
        node = module.PreviewNode.__new__(module.PreviewNode)
        node.markers_pub = Mock()
        odom = types.SimpleNamespace(header=types.SimpleNamespace(frame_id="odom"),
            pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=2, y=3))))
        node.publish_markers(odom, {"rf2o_yaw": 0, "pni_yaw": math.pi / 2,
                                   "fused_yaw": math.pi / 4, "aligned": True})
        markers = node.markers_pub.publish.call_args.args[0].markers
        self.assertEqual(len(markers), 3)
        self.assertEqual(markers[1].header.frame_id, "odom")
        self.assertEqual(markers[1].pose.position.x, 2)
        self.assertAlmostEqual(markers[1].pose.orientation.z, math.sin(math.pi / 4))
        self.assertEqual(markers[1].lifetime.nanosec, 500_000_000)
        node.publish_markers(odom, {"rf2o_yaw": 0, "pni_yaw": None,
                                   "fused_yaw": 0, "aligned": False})
        markers = node.markers_pub.publish.call_args.args[0].markers
        self.assertEqual([marker.action for marker in markers], [0, 2, 2])
        # ROS generated conversion code requires Python float even for DELETE
        # markers; a lax namespace stub otherwise hides a native assertion.
        for marker in markers:
            for field in ("x", "y", "z", "w"):
                self.assertIs(type(getattr(marker.pose.orientation, field)), float)
            for field in ("x", "y", "z"):
                self.assertIs(type(getattr(marker.pose.position, field)), float)
                self.assertIs(type(getattr(marker.scale, field)), float)
            for field in ("r", "g", "b", "a"):
                self.assertIs(type(getattr(marker.color, field)), float)


if __name__ == "__main__":
    unittest.main()
