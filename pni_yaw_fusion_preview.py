#!/usr/bin/env python3
"""Run alongside current mapping. Visualise yaw fusion without publishing TF."""
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from pni_yaw_fusion_core import YawFusion


def odom_yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite RF2O quaternion")
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-12:
        raise ValueError("invalid RF2O quaternion")
    x, y, z, w = [v / norm for v in values]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class PreviewNode(Node):
    def __init__(self):
        super().__init__("pni_yaw_fusion_preview")
        def parameter(name, default):
            return self.declare_parameter(name, default).value
        self.frame = parameter("odom_frame", "odom")
        self.base = parameter("base_frame", "base_link")
        self.filter = YawFusion(
            sign=parameter("heading_sign", 1),
            min_status=parameter("pni_min_status", 0),
            timeout=parameter("input_timeout", 0.5),
            pair_gap=parameter("pair_gap", 0.25),
            pni_std_deg=parameter("pni_std_deg", 8.0),
            drift_std_deg=parameter("rf2o_drift_std_deg", 2.0),
            max_innovation_deg=parameter("max_innovation_deg", 30.0),
        )
        self.status_pub = self.create_publisher(String, "/pni/yaw_fusion_preview", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/pni/yaw_comparison_markers", 10)
        self.create_subscription(String, parameter("pni_topic", "/stm32/sensor_json"),
                                 self.on_pni, qos_profile_sensor_data)
        self.create_subscription(Odometry, parameter("odom_topic", "/odom_rf2o"),
                                 self.on_odom, qos_profile_sensor_data)
        self.csv_file = self.csv_writer = None
        self.last_flush = self.last_log = 0.0
        csv_path = parameter("csv_path", "")
        self.columns = ("ros_time", "rf2o_deg", "pni_aligned_deg", "fused_deg",
                        "pni_status", "pni_used", "pni_fresh", "gain", "reason")
        if csv_path:
            path = Path(csv_path).expanduser()
            self.csv_file = path.open("a", newline="", encoding="utf-8")
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.columns)
            if self.csv_file.tell() == 0:
                self.csv_writer.writeheader()
        self.last_odom_rx = None
        self.create_timer(0.5, self.check_odom)
        self.get_logger().info(
            f"Yaw-only experiment: sign={self.filter.sign}, min_status={self.filter.min_status}; "
            "guarded fusion: current-input disagreement gate, shortest-arc bound, "
            "RF2O fallback on invalid/unpaired inputs; hold vehicle still during "
            "initial alignment. No TF is published. "
            + ("status=0 is accepted under the user's reliability assumption."
               if self.filter.min_status == 0 else "Low-quality PNI samples are filtered."))

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pni(self, message):
        try:
            data = json.loads(message.data)
            self.filter.ingest_pni(data, self.now_seconds())
        except (ValueError, TypeError) as exc:
            self.filter.pni = None
            self.filter.reason = f"invalid sensor JSON: {exc}"

    def on_odom(self, message):
        try:
            if message.header.frame_id != self.frame or message.child_frame_id != self.base:
                raise ValueError(f"expected {self.frame} -> {self.base}, got "
                                 f"{message.header.frame_id} -> {message.child_frame_id}")
            position = message.pose.pose.position
            if not all(math.isfinite(v) for v in (position.x, position.y, position.z)):
                raise ValueError("non-finite RF2O position")
            now = self.now_seconds()
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            result = self.filter.update_odom(odom_yaw(message.pose.pose.orientation), stamp, now)
            if result is None:
                return
            self.last_odom_rx = time.monotonic()
            result.update(ros_time=now, valid=True, frame_id=self.frame,
                          experimental=True, accepts_uncalibrated_pni=self.filter.min_status == 0)
            self.publish_status(result)
            self.publish_markers(message, result)
            row = {"ros_time": now, "rf2o_deg": math.degrees(result["rf2o_yaw"]),
                   "pni_aligned_deg": None if result["pni_yaw"] is None else math.degrees(result["pni_yaw"]),
                   "fused_deg": math.degrees(result["fused_yaw"]),
                   **{key: result[key] for key in ("pni_status", "pni_used", "pni_fresh", "gain", "reason")}}
            if self.csv_writer:
                self.csv_writer.writerow(row)
                if time.monotonic() - self.last_flush > 1:
                    self.csv_file.flush()
                    self.last_flush = time.monotonic()
            if time.monotonic() - self.last_log > 1:
                self.get_logger().info(json.dumps(row, ensure_ascii=False))
                self.last_log = time.monotonic()
        except ValueError as exc:
            self.publish_status({"valid": False, "experimental": True, "reason": str(exc)})

    def publish_status(self, data):
        message = String()
        message.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(message)

    def check_odom(self):
        if self.last_odom_rx is None or time.monotonic() - self.last_odom_rx > self.filter.timeout:
            self.publish_status({"valid": False, "experimental": True, "reason": "waiting for fresh RF2O odometry"})

    def publish_markers(self, odom, result):
        markers = MarkerArray()
        arrows = [
            (0, "RF2O_blue", result["rf2o_yaw"], (0.1, 0.4, 1.0), 0.55, 0.10),
            (1, "PNI_green", result["pni_yaw"], (0.1, 0.9, 0.3), 0.75, 0.18),
            (2, "Fused_orange", result["fused_yaw"] if result["aligned"] else None,
             (1.0, 0.5, 0.05), 0.95, 0.26),
        ]
        for ident, label, yaw, color, length, height in arrows:
            marker = Marker()
            marker.header = odom.header
            marker.ns, marker.id = label, ident
            marker.type = Marker.ARROW
            marker.action = Marker.DELETE if yaw is None else Marker.ADD
            marker.pose.position.x = float(odom.pose.pose.position.x)
            marker.pose.position.y = float(odom.pose.pose.position.y)
            marker.pose.position.z = height
            # ROS double fields must contain Python float, including DELETE
            # markers created before initial alignment or after PNI timeout.
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0 if yaw is None else math.sin(yaw / 2)
            marker.pose.orientation.w = 1.0 if yaw is None else math.cos(yaw / 2)
            marker.scale.x, marker.scale.y, marker.scale.z = length, 0.04, 0.04
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.9
            marker.lifetime.sec, marker.lifetime.nanosec = 0, 500_000_000
            markers.markers.append(marker)
        self.markers_pub.publish(markers)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = PreviewNode()
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
