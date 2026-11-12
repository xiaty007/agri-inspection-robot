"""Host-side checks for the STM32 receive path without Raspberry Pi packages."""

import json
import math
import struct
import sys
import time
import types
import unittest
import zlib
from unittest.mock import Mock, patch

from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stm32_serial_protocol import JsonLineFramer


def fake_module(name, **attributes):
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    sys.modules[name] = module
    return module


class Message:
    def __init__(self):
        self.data = ""
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.orientation = types.SimpleNamespace(x=0, y=0, z=0, w=0)
        self.linear_acceleration = types.SimpleNamespace(x=0, y=0, z=0)
        self.orientation_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9
        self.angular_velocity_covariance = [0.0] * 9


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: object())
        )

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)


class Flask:
    def __init__(self, _name):
        self.routes = {}

    def route(self, path, **_kwargs):
        def register(function):
            self.routes[path] = function
            return function
        return register


class Response:
    def __init__(self, body, mimetype=None):
        self.body = body
        self.mimetype = mimetype
        self.headers = {}


fake_module("cv2")
fake_module("flask", Flask=Flask, Response=Response, jsonify=lambda data: data,
            request=types.SimpleNamespace(host_url="http://rspi.local:5000/"))
fake_module("flask_cors", CORS=lambda _app: None)
fake_module("rclpy")
fake_module("rclpy.node", Node=Node)
for package, names in {
    "geometry_msgs": ("PoseStamped", "TransformStamped", "Vector3"),
    "nav_msgs": ("OccupancyGrid", "Odometry", "Path"),
    "sensor_msgs": ("Imu",),
    "std_msgs": ("Float32", "String"),
    "visualization_msgs": ("Marker",),
}.items():
    fake_module(package)
    fake_module(package + ".msg", **{name: Message for name in names})
fake_module("tf2_ros", TransformBroadcaster=Message)
fake_module("serial")

import raspberry_pi_temp_serial_service as flask_bridge
import stm32_pni_ros2_node as ros_bridge


def payload(tick, update_tick, heading=10.0, temperature=None):
    return {
        "tick": tick,
        "temperature": temperature,
        "unit": "C",
        "pni": {"valid": True, "heading": heading, "pitch": 0, "roll": 0,
                "status": 3, "update_tick": update_tick},
        "accel": {"valid": True, "x": 0, "y": 0, "z": 9.8,
                  "status": 3, "update_tick": update_tick},
    }


class ReceiveTests(unittest.TestCase):
    def test_homepage_lists_only_current_interfaces(self):
        home = flask_bridge.index()
        self.assertEqual(home.mimetype, "text/html")
        expected = {
            "/", "/sensor", "/snapshot", "/video_feed",
            "/map/meta", "/map/image", "/robot/pose",
            "/recognition/upload", "/recognition/image",
            "/recognition/status", "/debug/serial",
        }
        self.assertEqual(set(flask_bridge.app.routes), expected)
        for path in expected - {"/"}:
            self.assertIn(path, home.body)
        self.assertNotIn("/serialport", home.body)

    def test_occupancy_grid_png_and_metadata(self):
        pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=-1.0, y=2.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        message = types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="map"),
            info=types.SimpleNamespace(width=2, height=2, resolution=0.05, origin=pose),
            data=[0, 100, -1, 50],
        )
        flask_bridge.process_map_message(message)
        meta, status = flask_bridge.get_map_meta()
        self.assertEqual(status, 200)
        self.assertTrue(meta["success"])
        self.assertEqual(meta["data"]["frame_id"], "map")
        self.assertEqual(meta["data"]["origin"]["x"], -1.0)
        self.assertEqual(meta["data"]["resolution"], 0.05)

        response = flask_bridge.get_map_image()
        self.assertEqual(response.mimetype, "image/png")
        self.assertEqual(response.headers["Cache-Control"], "no-store")
        png = response.body
        self.assertTrue(png.startswith(b"\x89PNG\r\n\x1a\n"))
        offset = 8
        chunks = {}
        while offset < len(png):
            size = struct.unpack_from(">I", png, offset)[0]
            kind = png[offset + 4:offset + 8]
            chunks[kind] = png[offset + 8:offset + 8 + size]
            offset += size + 12
        self.assertEqual(struct.unpack_from(">II", chunks[b"IHDR"]), (2, 2))
        self.assertEqual(zlib.decompress(chunks[b"IDAT"]),
                         bytes([0, 205, 128, 0, 255, 0]))

    def test_split_lines_and_oversized_line(self):
        framer = JsonLineFramer(max_line_bytes=32)
        self.assertEqual(framer.feed(b'{"tick":1'), [])
        self.assertEqual(framer.feed(b'}\r\n{"tick":2}\n'), ['{"tick":1}', '{"tick":2}'])
        self.assertEqual(framer.feed(b"x" * 40 + b"\nOK\n"), ["OK"])
        self.assertEqual(framer.dropped_lines, 1)

    def test_null_temperature_keeps_pni_and_timeout_marks_offline(self):
        flask_bridge.last_valid_rx_time = None
        flask_bridge.sensor_data.update(packet_count=0, parse_error_count=0, tick=None, error="")
        self.assertTrue(flask_bridge.process_sensor_line(json.dumps(payload(1000, 950))))
        current = flask_bridge.sensor_snapshot()
        self.assertIsNone(current["temperature"])
        self.assertTrue(current["pni"]["valid"])
        self.assertTrue(current["online"])
        flask_bridge.last_valid_rx_time = time.monotonic() - 2.0
        stale = flask_bridge.sensor_snapshot()
        self.assertFalse(stale["online"])
        self.assertFalse(stale["pni"]["valid"])

    def make_ros_node(self):
        node = ros_bridge.Stm32PniNode.__new__(ros_bridge.Stm32PniNode)
        for name in ("raw_pub", "status_pub", "heading_pub", "rpy_pub", "imu_pub"):
            setattr(node, name, Publisher())
        node.packet_count = 0
        node.parse_error_count = 0
        node.pni_valid_count = 0
        node.rejected_packet_count = 0
        node.last_reject_reason = ""
        node.last_log_time = time.monotonic()
        node.last_packet_time = None
        node.stale_reported = False
        node.last_seen_pni_tick = None
        node.last_seen_accel_tick = None
        node.last_pni_accepted = False
        node.last_accel_accepted = False
        node.last_heading_deg = None
        node.reacquire_heading_deg = None
        node.reacquire_count = 0
        node.calibration_done = False
        node.accel_bias_x = 0
        node.accel_bias_y = 0
        node.last_orientation = None
        return node

    def send(self, node, data):
        node.packet_count += 1
        node.publish_data(data, json.dumps(data))

    def test_repeat_measurement_is_not_republished(self):
        node = self.make_ros_node()
        self.send(node, payload(1000, 950))
        self.send(node, payload(1050, 950))
        self.assertEqual(len(node.heading_pub.messages), 1)
        status = json.loads(node.status_pub.messages[-1].data)
        self.assertTrue(status["pni_valid"])
        self.assertFalse(status["pni_new"])
        self.send(node, payload(1100, 1100))
        self.assertEqual(len(node.heading_pub.messages), 2)
        status_count = len(node.status_pub.messages)
        node.last_packet_time = time.monotonic() - 2.0
        node.check_stale()
        node.check_stale()
        self.assertEqual(len(node.status_pub.messages), status_count + 1)
        self.assertFalse(json.loads(node.status_pub.messages[-1].data)["online"])
        self.assertEqual(len(node.heading_pub.messages), 2)

    def test_heading_reacquires_and_old_measurement_is_rejected(self):
        node = self.make_ros_node()
        self.send(node, payload(1000, 1000, 10))
        self.send(node, payload(1100, 1100, 100))
        self.send(node, payload(1200, 1200, 101))
        self.assertEqual(len(node.heading_pub.messages), 1)
        self.send(node, payload(1300, 1300, 102))
        self.assertEqual(len(node.heading_pub.messages), 2)
        self.send(node, payload(2000, 1300, 102))
        self.assertFalse(json.loads(node.status_pub.messages[-1].data)["pni_valid"])
        self.assertEqual(len(node.heading_pub.messages), 2)


class RobotPoseTests(unittest.TestCase):
    def setUp(self):
        self.previous_map = dict(flask_bridge.map_state)
        self.previous_pose = dict(flask_bridge.robot_pose_state)
        self.previous_rx = flask_bridge.robot_pose_rx_time
        flask_bridge.robot_pose_state.clear()
        flask_bridge.robot_pose_state.update(valid=False, error="TF not received yet")
        flask_bridge.robot_pose_rx_time = None
        flask_bridge.map_state.update(
            available=True, frame_id="map", width=100, height=80,
            resolution=0.1, origin={"x": -1.0, "y": -2.0, "yaw": 0.0}, version=7,
        )

    def tearDown(self):
        flask_bridge.map_state.clear()
        flask_bridge.map_state.update(self.previous_map)
        flask_bridge.robot_pose_state.clear()
        flask_bridge.robot_pose_state.update(self.previous_pose)
        flask_bridge.robot_pose_rx_time = self.previous_rx

    def transform(self, x=1.0, y=1.0, yaw=math.pi / 2, seconds=100):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="map", stamp=types.SimpleNamespace(sec=seconds, nanosec=0)),
            child_frame_id="base_link",
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=x, y=y, z=0.0),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        )

    def test_live_pose_and_png_coordinates(self):
        flask_bridge.process_robot_transform(self.transform(), 100.05)
        response, status, headers = flask_bridge.get_robot_pose()
        self.assertEqual(status, 200)
        self.assertEqual(headers["Cache-Control"], "no-store")
        data = response["data"]
        self.assertTrue(response["success"])
        self.assertAlmostEqual(data["yaw_deg"], 90)
        self.assertEqual(data["pixel"]["x"], 20)
        self.assertEqual(data["pixel"]["y"], 50)
        self.assertAlmostEqual(data["pixel"]["yaw"], -math.pi / 2)
        self.assertEqual(data["pixel"]["map_version"], 7)
        self.assertTrue(data["pixel"]["in_bounds"])

    def test_rotated_origin_and_current_map_geometry(self):
        flask_bridge.map_state["origin"] = {"x": 1.0, "y": 2.0, "yaw": math.pi / 2}
        flask_bridge.process_robot_transform(self.transform(x=1.0, y=3.0), 100)
        pixel = flask_bridge.robot_pose_snapshot()["pixel"]
        self.assertAlmostEqual(pixel["x"], 10)
        self.assertAlmostEqual(pixel["y"], 80)
        self.assertAlmostEqual(pixel["yaw"], 0)
        flask_bridge.map_state.update(version=8, origin={"x": 0.0, "y": 0.0, "yaw": 0.0})
        pixel = flask_bridge.robot_pose_snapshot()["pixel"]
        self.assertEqual(pixel["map_version"], 8)
        self.assertAlmostEqual(pixel["x"], 10)
        self.assertAlmostEqual(pixel["y"], 50)

    def test_stale_snapshot_clears_coordinates(self):
        with patch.object(flask_bridge.time, "monotonic", return_value=20):
            flask_bridge.process_robot_transform(self.transform(), 100.2)
        with patch.object(flask_bridge.time, "monotonic", return_value=21):
            response, status, _ = flask_bridge.get_robot_pose()
        self.assertEqual(status, 503)
        self.assertFalse(response["success"])
        for field in ("x", "y", "z", "yaw", "pixel"):
            self.assertIsNone(response["data"][field])

    def test_missing_stale_future_and_invalid_tf(self):
        response, status, _ = flask_bridge.get_robot_pose()
        self.assertEqual(status, 503)
        for seconds, now in ((0, 100), (100, 102), (102, 100)):
            with self.assertRaises(ValueError):
                flask_bridge.process_robot_transform(self.transform(seconds=seconds), now)
        with self.assertRaises(ValueError):
            flask_bridge.process_robot_transform(self.transform(x=float("nan")), 100)
        transform = self.transform()
        transform.transform.rotation.z = transform.transform.rotation.w = 0
        with self.assertRaises(ValueError):
            flask_bridge.process_robot_transform(transform, 100)

    def test_map_missing_frame_mismatch_and_outside(self):
        flask_bridge.process_robot_transform(self.transform(x=20), 100)
        self.assertFalse(flask_bridge.robot_pose_snapshot()["pixel"]["in_bounds"])
        flask_bridge.map_state["available"] = False
        data = flask_bridge.robot_pose_snapshot()
        self.assertTrue(data["valid"])
        self.assertIsNone(data["pixel"])
        flask_bridge.map_state.update(available=True, frame_id="other_map")
        self.assertIsNone(flask_bridge.robot_pose_snapshot()["pixel"])

    def test_tf_poller_uses_target_map_and_handles_loss(self):
        calls = []
        transform = self.transform()
        def lookup(*args):
            calls.append(args)
            return transform
        buffer = types.SimpleNamespace(lookup_transform=lookup)
        node = types.SimpleNamespace(get_clock=lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=100_000_000_000)))
        latest = object()
        flask_bridge.poll_robot_transform(buffer, node, latest, LookupError)
        self.assertEqual(calls, [("map", "base_link", latest)])
        self.assertTrue(flask_bridge.robot_pose_snapshot()["valid"])
        def missing(*_args):
            raise LookupError("TF chain missing")
        buffer.lookup_transform = missing
        flask_bridge.poll_robot_transform(buffer, node, latest, LookupError)
        response, status, _ = flask_bridge.get_robot_pose()
        self.assertEqual(status, 503)
        self.assertIn("TF chain missing", response["data"]["error"])
        self.assertIsNone(response["data"]["x"])

    def test_ros_loop_registers_tf_and_timer_and_cleans_up(self):
        callbacks = []
        node = Mock()
        node.get_clock.return_value.now.return_value.nanoseconds = 100_000_000_000
        node.create_timer.side_effect = lambda period, callback: callbacks.append((period, callback))
        buffer = Mock()
        buffer.lookup_transform.return_value = self.transform()
        listener = Mock()
        ros = types.ModuleType("rclpy")
        ros.init = Mock()
        ros.ok = lambda: True
        ros.shutdown = Mock()
        ros.spin = lambda _node: callbacks[0][1]()
        replacements = {
            "rclpy": ros,
            "rclpy.node": types.SimpleNamespace(Node=Mock(return_value=node)),
            "rclpy.signals": types.SimpleNamespace(SignalHandlerOptions=types.SimpleNamespace(NO=0)),
            "rclpy.time": types.SimpleNamespace(Time=lambda: "latest"),
            "tf2_ros": types.SimpleNamespace(TransformException=LookupError),
            "tf2_ros.buffer": types.SimpleNamespace(Buffer=Mock(return_value=buffer)),
            "tf2_ros.transform_listener": types.SimpleNamespace(TransformListener=Mock(return_value=listener)),
        }
        with patch.dict(sys.modules, replacements), patch.object(
                flask_bridge.time, "sleep", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                flask_bridge.ros_read_loop()
        self.assertEqual([call.args[1] for call in node.create_subscription.call_args_list],
                         ["/stm32/sensor_json", "/map"])
        self.assertAlmostEqual(callbacks[0][0], 1 / flask_bridge.ROBOT_POSE_RATE)
        buffer.lookup_transform.assert_called_once_with("map", "base_link", "latest")
        self.assertTrue(flask_bridge.robot_pose_snapshot()["valid"])
        listener.unregister.assert_called_once()
        node.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
