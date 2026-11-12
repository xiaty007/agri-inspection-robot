#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from stm32_serial_protocol import JsonLineFramer

try:
    import serial
except ImportError as exc:
    raise SystemExit("Missing pyserial. Install with: sudo apt install python3-serial") from exc


SERIAL_PORT = os.environ.get("STM32_SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("STM32_BAUD_RATE", "115200"))
FRAME_ID = os.environ.get("PNI_FRAME_ID", "base_link")
ODOM_FRAME_ID = os.environ.get("PNI_ODOM_FRAME_ID", "odom")
PATH_MAX_LEN = int(os.environ.get("PNI_PATH_MAX_LEN", "1000"))
ACCEL_DEADBAND = float(os.environ.get("PNI_ACCEL_DEADBAND", "0.12"))
VELOCITY_DAMPING = float(os.environ.get("PNI_VELOCITY_DAMPING", "0.98"))
MAX_DT = float(os.environ.get("PNI_MAX_DT", "0.2"))
MAX_SPEED = float(os.environ.get("PNI_MAX_SPEED", "1.5"))
ENABLE_ACCEL_POSITION = os.environ.get("PNI_ENABLE_ACCEL_POSITION", "0") == "1"
CALIBRATION_SECONDS = float(os.environ.get("PNI_CALIBRATION_SECONDS", "2.0"))
STATIONARY_SPEED_EPS = float(os.environ.get("PNI_STATIONARY_SPEED_EPS", "0.03"))
MAX_HEADING_JUMP_DEG = float(os.environ.get("PNI_MAX_HEADING_JUMP_DEG", "45.0"))
MAX_ACCEL_ABS = float(os.environ.get("PNI_MAX_ACCEL_ABS", "20.0"))
MAX_ACCEL_NORM = float(os.environ.get("PNI_MAX_ACCEL_NORM", "20.0"))
MAX_DATA_AGE_MS = int(os.environ.get("PNI_MAX_DATA_AGE_MS", "500"))
MIN_PNI_STATUS = int(os.environ.get("PNI_MIN_STATUS", "0"))
PUBLISH_ODOM = os.environ.get("PNI_PUBLISH_ODOM", "0") == "1"
PUBLISH_TF = os.environ.get("PNI_PUBLISH_TF", "0") == "1"
PUBLISH_MARKER = os.environ.get("PNI_PUBLISH_MARKER", "0") == "1"
REACQUIRE_PACKETS = int(os.environ.get("PNI_REACQUIRE_PACKETS", "3"))
RX_TIMEOUT = float(os.environ.get("STM32_SENSOR_TIMEOUT", "1.0"))


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def angle_diff_deg(current, previous):
    return (current - previous + 180.0) % 360.0 - 180.0


def finite_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


class Stm32PniNode(Node):
    def __init__(self):
        super().__init__("stm32_pni_node")
        self.heading_pub = self.create_publisher(Float32, "/pni/heading", 10)
        self.rpy_pub = self.create_publisher(Vector3, "/pni/rpy", 10)
        self.imu_pub = self.create_publisher(Imu, "/pni/imu", 10)
        self.odom_pub = self.create_publisher(Odometry, "/pni/odom", 10)
        self.path_pub = self.create_publisher(Path, "/pni/path", 10)
        self.raw_pub = self.create_publisher(String, "/stm32/sensor_json", 10)
        self.status_pub = self.create_publisher(String, "/pni/status", 10)
        self.marker_pub = self.create_publisher(Marker, "/pni/heading_marker", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_imu_time = None
        self.position_x = 0.0
        self.position_y = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_orientation = None
        self.calibration_start_time = None
        self.calibration_done = False
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.accel_bias_count = 0
        self.accel_bias_sum_x = 0.0
        self.accel_bias_sum_y = 0.0
        self.last_heading_deg = None
        self.reacquire_heading_deg = None
        self.reacquire_count = 0
        self.last_seen_pni_tick = None
        self.last_seen_accel_tick = None
        self.last_pni_accepted = False
        self.last_accel_accepted = False
        self.last_packet_time = None
        self.stale_reported = False
        self.rejected_packet_count = 0
        self.last_reject_reason = ""
        self.path = Path()
        self.path.header.frame_id = ODOM_FRAME_ID
        self.last_odom_stamp = None

        self.packet_count = 0
        self.parse_error_count = 0
        self.pni_valid_count = 0
        self.last_status = "starting"
        self.last_log_time = time.monotonic()
        self.stop_event = threading.Event()
        self.create_timer(0.2, self.check_stale)
        self.reader_thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.reader_thread.start()

    def destroy_node(self):
        self.stop_event.set()
        super().destroy_node()

    def serial_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                self.get_logger().info(f"Opening STM32 serial {SERIAL_PORT} @ {BAUD_RATE}")
                with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05) as ser:
                    ser.reset_input_buffer()
                    self.get_logger().info("STM32 serial opened")
                    framer = JsonLineFramer()

                    while rclpy.ok() and not self.stop_event.is_set():
                        for line in framer.feed(ser.read(256)):
                            try:
                                data = json.loads(line)
                                if not isinstance(data, dict):
                                    raise ValueError("JSON root must be an object")
                                self.packet_count += 1
                                self.publish_data(data, line)
                            except (ValueError, TypeError, KeyError) as exc:
                                self.parse_error_count += 1
                                if time.monotonic() - self.last_log_time >= 1.0:
                                    self.last_log_time = time.monotonic()
                                    self.get_logger().warn(
                                        f"parse failed count={self.parse_error_count}: {exc}; line={line[:80]}"
                                    )
            except Exception as exc:
                self.get_logger().error(f"serial error: {exc}")
                time.sleep(2.0)

    def publish_data(self, data, line):
        now = self.get_clock().now().to_msg()
        pni = data.get("pni") or {}
        accel = data.get("accel") or {}
        if not isinstance(pni, dict) or not isinstance(accel, dict):
            raise ValueError("pni and accel must be objects")
        self.last_packet_time = time.monotonic()
        self.stale_reported = False

        raw_msg = String()
        raw_msg.data = line
        self.raw_pub.publish(raw_msg)

        pni_tick = pni.get("update_tick")
        accel_tick = accel.get("update_tick")
        accel_reported_valid = bool(accel.get("valid", False))
        pni_repeated = pni_tick is not None and pni_tick == self.last_seen_pni_tick
        accel_repeated = accel_tick is not None and accel_tick == self.last_seen_accel_tick
        if pni_tick is not None:
            self.last_seen_pni_tick = pni_tick
        if accel_tick is not None:
            self.last_seen_accel_tick = accel_tick
        if pni_repeated:
            age = self.measurement_age(data.get("tick"), pni_tick)
            pni_valid = bool(pni.get("valid", False)) and self.last_pni_accepted and age is not None and age <= MAX_DATA_AGE_MS
            reject_reason = "" if pni_valid else "repeated stale measurement"
        else:
            pni_valid, reject_reason = self.validate_pni_packet(pni, data.get("tick"))
            self.last_pni_accepted = pni_valid
        if accel_repeated:
            age = self.measurement_age(data.get("tick"), accel_tick)
            accel_valid = bool(accel.get("valid", False)) and self.last_accel_accepted and age is not None and age <= MAX_DATA_AGE_MS
            accel_reject_reason = "" if accel_valid else "repeated stale measurement"
        else:
            accel_valid, accel_reject_reason = self.validate_accel_packet(accel, data.get("tick"))
            self.last_accel_accepted = accel_valid
        if not accel_valid or accel_repeated:
            accel = dict(accel)
            accel["valid"] = False

        if not pni_valid and reject_reason and not pni_repeated and pni.get("valid", False):
            self.rejected_packet_count += 1
            self.last_reject_reason = reject_reason
        elif not accel_valid and accel_reject_reason and not accel_repeated and accel_reported_valid:
            self.rejected_packet_count += 1
            self.last_reject_reason = accel_reject_reason

        if pni_valid and not pni_repeated:
            self.pni_valid_count += 1
        self.publish_status(data, pni_valid, accel_valid, pni, not pni_repeated, not accel_repeated)

        if time.monotonic() - self.last_log_time >= 1.0:
            self.last_log_time = time.monotonic()
            self.get_logger().info(
                f"packets={self.packet_count}, pni_valid={pni_valid}, "
                f"accel_valid={accel_valid}, pni_ok_packets={self.pni_valid_count}"
            )

        if not pni_valid or pni_repeated:
            return

        heading_deg = float(pni.get("heading", 0.0))
        pitch_deg = float(pni.get("pitch", 0.0))
        roll_deg = float(pni.get("roll", 0.0))

        heading_msg = Float32()
        heading_msg.data = heading_deg
        self.heading_pub.publish(heading_msg)

        rpy_msg = Vector3()
        rpy_msg.x = roll_deg
        rpy_msg.y = pitch_deg
        rpy_msg.z = heading_deg
        self.rpy_pub.publish(rpy_msg)

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(heading_deg)
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        self.last_orientation = (qx, qy, qz, qw, yaw)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = FRAME_ID
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.orientation_covariance[0] = 0.05
        imu.orientation_covariance[4] = 0.05
        imu.orientation_covariance[8] = 0.05

        if accel.get("valid", False):
            imu.linear_acceleration.x = float(accel.get("x", 0.0))
            imu.linear_acceleration.y = float(accel.get("y", 0.0))
            imu.linear_acceleration.z = float(accel.get("z", 0.0))
            imu.linear_acceleration_covariance[0] = 0.2
            imu.linear_acceleration_covariance[4] = 0.2
            imu.linear_acceleration_covariance[8] = 0.2
        else:
            imu.linear_acceleration_covariance[0] = -1.0

        imu.angular_velocity_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        self.update_inertial_odom(now, yaw, qx, qy, qz, qw, accel)
        if PUBLISH_MARKER:
            self.publish_heading_marker(now, yaw)

    @staticmethod
    def measurement_age(transport_tick, update_tick):
        if (not isinstance(transport_tick, int) or isinstance(transport_tick, bool)
                or not isinstance(update_tick, int) or isinstance(update_tick, bool)
                or not 0 <= transport_tick <= 0xFFFFFFFF
                or not 0 <= update_tick <= 0xFFFFFFFF):
            return None
        return (transport_tick - update_tick) & 0xFFFFFFFF

    def validate_pni_packet(self, pni, transport_tick):
        if not bool(pni.get("valid", False)):
            return False, "pni.valid false"

        status = int(pni.get("status", 0))
        if not MIN_PNI_STATUS <= status <= 3:
            return False, f"pni status below quality threshold or invalid: {status}"
        age = self.measurement_age(transport_tick, pni.get("update_tick"))
        if age is None or age > MAX_DATA_AGE_MS:
            return False, f"pni stale or missing update_tick: {age}"

        heading = finite_float(pni.get("heading"))
        pitch = finite_float(pni.get("pitch"))
        roll = finite_float(pni.get("roll"))
        if heading is None or pitch is None or roll is None:
            return False, "pni angle not finite"

        if heading < -180.0 or heading > 180.0:
            return False, f"heading out of range: {heading:.2f}"
        if pitch < -90.0 or pitch > 90.0:
            return False, f"pitch out of range: {pitch:.2f}"
        if roll < -90.0 or roll > 90.0:
            return False, f"roll out of range: {roll:.2f}"

        if self.last_heading_deg is not None:
            jump = abs(angle_diff_deg(heading, self.last_heading_deg))
            if jump > MAX_HEADING_JUMP_DEG:
                if (self.reacquire_heading_deg is not None
                        and abs(angle_diff_deg(heading, self.reacquire_heading_deg)) <= MAX_HEADING_JUMP_DEG):
                    self.reacquire_count += 1
                else:
                    self.reacquire_count = 1
                self.reacquire_heading_deg = heading
                if self.reacquire_count < max(2, REACQUIRE_PACKETS):
                    return False, f"heading jump {jump:.2f} deg; reacquire {self.reacquire_count}"

        self.last_heading_deg = heading
        self.reacquire_heading_deg = None
        self.reacquire_count = 0
        return True, ""

    def validate_accel_packet(self, accel, transport_tick):
        if not bool(accel.get("valid", False)):
            return False, "accel.valid false"
        age = self.measurement_age(transport_tick, accel.get("update_tick"))
        if age is None or age > MAX_DATA_AGE_MS:
            return False, f"accel stale or missing update_tick: {age}"

        ax = finite_float(accel.get("x"))
        ay = finite_float(accel.get("y"))
        az = finite_float(accel.get("z"))
        if ax is None or ay is None or az is None:
            return False, "accel not finite"

        if max(abs(ax), abs(ay), abs(az)) > MAX_ACCEL_ABS:
            return False, f"accel axis too large: x={ax:.2f}, y={ay:.2f}, z={az:.2f}"

        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > MAX_ACCEL_NORM:
            return False, f"accel norm too large: {norm:.2f}"

        return True, ""

    def publish_status(self, data, pni_valid, accel_valid, pni, pni_new=False, accel_new=False):
        status = {
            "packets": self.packet_count,
            "parse_errors": self.parse_error_count,
            "rejected_packets": self.rejected_packet_count,
            "last_reject_reason": self.last_reject_reason,
            "pni_valid": pni_valid,
            "accel_valid": accel_valid,
            "pni_new": pni_valid and pni_new,
            "accel_new": accel_valid and accel_new,
            "online": self.last_packet_time is not None and time.monotonic() - self.last_packet_time <= RX_TIMEOUT,
            "pni_update_tick": pni.get("update_tick"),
            "accel_position_enabled": ENABLE_ACCEL_POSITION,
            "calibration_done": self.calibration_done,
            "accel_bias_x": round(self.accel_bias_x, 4),
            "accel_bias_y": round(self.accel_bias_y, 4),
            "tick": data.get("tick"),
            "temperature": data.get("temperature"),
            "heading": pni.get("heading"),
            "pitch": pni.get("pitch"),
            "roll": pni.get("roll"),
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    def update_inertial_odom(self, stamp, yaw, qx, qy, qz, qw, accel):
        if not PUBLISH_ODOM:
            return
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            self.publish_odom_and_path(stamp, qx, qy, qz, qw)
            return

        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time
        if dt <= 0.0 or dt > MAX_DT:
            return

        if not ENABLE_ACCEL_POSITION:
            self.velocity_x = 0.0
            self.velocity_y = 0.0
            self.publish_odom_and_path(stamp, qx, qy, qz, qw)
            return

        if not accel.get("valid", False):
            self.publish_odom_and_path(stamp, qx, qy, qz, qw)
            return

        ax_body = float(accel.get("x", 0.0))
        ay_body = float(accel.get("y", 0.0))

        ax_world = ax_body * math.cos(yaw) - ay_body * math.sin(yaw)
        ay_world = ax_body * math.sin(yaw) + ay_body * math.cos(yaw)

        if abs(ax_world) < ACCEL_DEADBAND:
            ax_world = 0.0
        if abs(ay_world) < ACCEL_DEADBAND:
            ay_world = 0.0

        self.velocity_x = (self.velocity_x + ax_world * dt) * VELOCITY_DAMPING
        self.velocity_y = (self.velocity_y + ay_world * dt) * VELOCITY_DAMPING

        speed = math.hypot(self.velocity_x, self.velocity_y)
        if speed > MAX_SPEED:
            scale = MAX_SPEED / speed
            self.velocity_x *= scale
            self.velocity_y *= scale

        if ax_world == 0.0 and ay_world == 0.0 and speed < 0.03:
            self.velocity_x = 0.0
            self.velocity_y = 0.0

        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt
        self.publish_odom_and_path(stamp, qx, qy, qz, qw)

    def publish_odom_and_path(self, stamp, qx, qy, qz, qw):
        self.last_odom_stamp = stamp
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = ODOM_FRAME_ID
        odom.child_frame_id = FRAME_ID
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.velocity_x
        odom.twist.twist.linear.y = self.velocity_y
        odom.pose.covariance[0] = 1.0
        odom.pose.covariance[7] = 1.0
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 2.0
        odom.twist.covariance[7] = 2.0
        self.odom_pub.publish(odom)
        if PUBLISH_TF:
            self.publish_odom_tf(stamp, qx, qy, qz, qw)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = ODOM_FRAME_ID
        pose.pose = odom.pose.pose

        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > PATH_MAX_LEN:
            self.path.poses = self.path.poses[-PATH_MAX_LEN:]
        self.path_pub.publish(self.path)

    def check_stale(self):
        if (self.last_packet_time is not None
                and time.monotonic() - self.last_packet_time > RX_TIMEOUT
                and not self.stale_reported):
            self.stale_reported = True
            self.publish_status({}, False, False, {})

    def publish_odom_tf(self, stamp, qx, qy, qz, qw):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = ODOM_FRAME_ID
        transform.child_frame_id = FRAME_ID
        transform.transform.translation.x = self.position_x
        transform.transform.translation.y = self.position_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def publish_heading_marker(self, stamp, yaw):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = ODOM_FRAME_ID
        marker.ns = "pni"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.position_x
        marker.pose.position.y = self.position_y
        marker.pose.position.z = 0.15
        qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 1.2
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        dot = Marker()
        dot.header.stamp = stamp
        dot.header.frame_id = ODOM_FRAME_ID
        dot.ns = "pni"
        dot.id = 2
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position.x = self.position_x
        dot.pose.position.y = self.position_y
        dot.pose.position.z = 0.08
        dot.scale.x = 0.18
        dot.scale.y = 0.18
        dot.scale.z = 0.18
        dot.color.r = 1.0
        dot.color.g = 0.2
        dot.color.b = 0.1
        dot.color.a = 1.0
        self.marker_pub.publish(dot)


def main():
    rclpy.init()
    node = Stm32PniNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
