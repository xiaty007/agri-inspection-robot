#!/usr/bin/env python3
"""Experimental stationary-origin cloud. No serial ownership, odometry or TF."""
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

from pni_tilt_cloud_core import TiltCloud


def publish_period(rate):
    if not isinstance(rate, (int, float)) or isinstance(rate, bool) or not (
            math.isfinite(rate) and 1.0 <= rate <= 30.0):
        raise ValueError("publish_rate_hz must be between 1 and 30 Hz")
    return 1.0 / rate


class TiltCloudNode(Node):
    def __init__(self):
        super().__init__("pni_tilt_cloud_preview")
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.frame = param("reference_frame", "scan_reference")
        self.laser_frame = param("laser_frame", "laser")
        self.core = TiltCloud(
            voxel_size=param("voxel_size", 0.03), max_points=param("max_points", 50000),
            settle_time=param("settle_time", 0.6), max_rate_deg=param("max_rate_deg", 5.0),
            timeout=param("input_timeout", 0.5), pair_gap=param("pair_gap", 0.15),
            use_heading=param("use_heading", False), roll_sign=param("roll_sign", 1),
            pitch_sign=param("pitch_sign", 1), min_status=param("pni_min_status", 0),
            mount_rpy_deg=(param("mount_roll_deg", 0.0), param("mount_pitch_deg", 0.0),
                           param("mount_yaw_deg", 0.0)),
            sensor_height=param("sensor_height", 0.0), max_range=param("max_range", 8.0))
        self.save_dir = Path(param("save_dir", "~/tilt_scan_results")).expanduser()
        self.pending = deque(maxlen=5)
        self.last_scan_rx = None
        self.last_status_publish = 0.0
        self.cloud_pub = self.create_publisher(PointCloud2, "/pni_scan_cloud", 1)
        self.status_pub = self.create_publisher(String, "/tilt_scan/status", 10)
        self.create_subscription(String, "/stm32/sensor_json", self.on_pni, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_service(Empty, "/tilt_scan/clear", self.clear)
        self.create_service(Trigger, "/tilt_scan/save", self.save)
        self.create_timer(0.05, self.process_scans)
        self.create_timer(publish_period(param("publish_rate_hz", 10.0)), self.publish)
        self.get_logger().info(
            "Tilt preview only: fixed sensor origin, stop-and-capture; NO TF/odometry. "
            "Default ignores yaw: do not turn left/right. PNI/laser axes must be calibrated. "
            "status=0 is accepted for this experiment. Hold still >=1 second at each tilt.")

    def seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pni(self, message):
        try:
            self.core.ingest(json.loads(message.data), self.seconds())
        except (ValueError, TypeError) as exc:
            self.core.input_valid = False
            self.core.reason = f"invalid JSON: {exc}"

    def on_scan(self, message):
        self.last_scan_rx = time.monotonic()
        if message.header.frame_id != self.laser_frame:
            self.core.reason = f"unexpected scan frame: {message.header.frame_id}"
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if message.time_increment > 0:
            duration = message.time_increment * max(0, len(message.ranges) - 1)
        else:
            duration = message.scan_time  # Conservative duration fallback.
        if not math.isfinite(duration) or duration < 0:
            self.core.reason = "invalid scan duration"
            return
        self.pending.append((stamp, duration, message))

    def process_scans(self):
        now = self.seconds()
        while self.pending:
            start, duration, scan = self.pending[0]
            # Delay a little for the PNI sample covering the end of the scan.
            if start + duration + 0.08 > now and start <= now + 0.15:
                break
            self.pending.popleft()
            self.core.add_scan(scan.ranges, scan.angle_min, scan.angle_increment,
                               scan.range_min, scan.range_max, start, duration, now)

    def cloud_message(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.height, msg.width = 1, len(self.core.voxels)
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(("x", "y", "z"))]
        msg.is_bigendian, msg.is_dense = False, True
        msg.point_step, msg.row_step = 12, 12 * msg.width
        msg.data = self.core.packed_points()
        return msg

    def publish(self):
        self.cloud_pub.publish(self.cloud_message())
        # Cloud refresh is configurable; diagnostics remain at about 2 Hz.
        current = time.monotonic()
        if current - self.last_status_publish < 0.5:
            return
        self.last_status_publish = current
        reason = self.core.reason
        if self.last_scan_rx is None or time.monotonic() - self.last_scan_rx > self.core.timeout:
            reason = "waiting for fresh /scan; old accumulated points retained"
        elif not self.core.history or self.seconds() - self.core.history[-1][0] > self.core.timeout:
            reason = "waiting for fresh PNI; old accumulated points retained"
        message = String()
        message.data = json.dumps({"experimental": True, "fixed_origin_assumption": True,
            "frame_id": self.frame, "points": len(self.core.voxels),
            "accepted_scans": self.core.accepted_scans, "reason": reason,
            "use_heading": self.core.use_heading, "sensor_height": self.core.sensor_height})
        self.status_pub.publish(message)

    def clear(self, request, response):
        self.core.clear()
        self.pending.clear()
        self.last_status_publish = 0.0
        self.publish()
        return response

    def save(self, request, response):
        if not self.core.voxels:
            response.success, response.message = False, "No points to save"
            return response
        try:
            self.save_dir.mkdir(parents=True, exist_ok=True)
            path = self.save_dir / ("tilt_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".ply")
            with path.open("x", encoding="ascii", newline="\n") as stream:
                stream.write("ply\nformat ascii 1.0\n")
                stream.write(f"comment frame {self.frame}\ncomment experimental fixed sensor origin; units meters\n")
                stream.write(f"element vertex {len(self.core.voxels)}\n")
                stream.write("property float x\nproperty float y\nproperty float z\nend_header\n")
                for x, y, z in self.core.voxels.values():
                    stream.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
            response.success, response.message = True, str(path)
        except (OSError, UnicodeError) as exc:
            response.success, response.message = False, str(exc)
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = TiltCloudNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
