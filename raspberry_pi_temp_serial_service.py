#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Expose STM32 sensor JSON to a mini program, normally via the ROS2 topic.

Expected STM32 line:
{"tick":12345,"temperature":28.56,"unit":"C","pni":{...},"accel":{...}}
"""

import json
import math
import os
import queue
import struct
import threading
import time
import zlib
from datetime import datetime

import cv2
from flask import Flask, Response, jsonify, request
from flask_cors import CORS
from stm32_serial_protocol import JsonLineFramer


DATA_SOURCE = os.environ.get("STM32_DATA_SOURCE", "ros").lower()
SERIAL_PORT = os.environ.get("STM32_SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("STM32_BAUD_RATE", "115200"))
SERIAL_TIMEOUT = float(os.environ.get("STM32_SERIAL_TIMEOUT", "0.05"))
SERIAL_LOG_INTERVAL = float(os.environ.get("STM32_SERIAL_LOG_INTERVAL", "1.0"))
SENSOR_TIMEOUT = float(os.environ.get("STM32_SENSOR_TIMEOUT", "1.0"))
SERVICE_PORT = int(os.environ.get("STM32_SERVICE_PORT", "5000"))
PUBLIC_HOST = os.environ.get("STM32_PUBLIC_HOST", "rspi.local")
CAMERA_INDEX = int(os.environ.get("CAMERA_INDEX", "0"))
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
CAMERA_FPS = int(os.environ.get("CAMERA_FPS", "15"))
ROBOT_BASE_FRAME = os.environ.get("ROBOT_BASE_FRAME", "base_link")
ROBOT_MAP_FRAME = os.environ.get("ROBOT_MAP_FRAME", "map")
ROBOT_POSE_TIMEOUT = float(os.environ.get("ROBOT_POSE_TIMEOUT", "1.0"))
ROBOT_POSE_RATE = float(os.environ.get("ROBOT_POSE_RATE", "10"))
if ROBOT_POSE_TIMEOUT <= 0 or not math.isfinite(ROBOT_POSE_TIMEOUT):
    raise ValueError("ROBOT_POSE_TIMEOUT must be positive and finite")
if ROBOT_POSE_RATE <= 0 or not math.isfinite(ROBOT_POSE_RATE):
    raise ValueError("ROBOT_POSE_RATE must be positive and finite")

app = Flask(__name__)
CORS(app)

camera = None
camera_lock = threading.RLock()
recognition_lock = threading.Lock()
sensor_lock = threading.Lock()
map_lock = threading.Lock()
robot_pose_lock = threading.Lock()
robot_pose_state = {"valid": False, "error": "TF not received yet"}
robot_pose_rx_time = None
map_queue = queue.Queue(maxsize=1)
last_valid_rx_time = None
map_image = None
map_state = {
    "available": False,
    "version": 0,
    "timestamp": "",
    "frame_id": "",
    "width": 0,
    "height": 0,
    "resolution": None,
    "origin": None,
    "image_url": "",
    "error": "",
}

# OccupancyGrid: 0 = free, 100 = occupied, 255 = unknown (-1 as uint8).
MAP_PALETTE = bytes(
    205 if value > 100 else round((100 - value) * 255 / 100)
    for value in range(256)
)
recognition_image = None
recognition_data = {
    "available": False,
    "timestamp": "",
    "size": 0,
    "content_type": "image/jpeg",
    "error": "",
}
sensor_data = {
    "temperature": None,
    "unit": "C",
    "timestamp": "",
    "online": False,
    "last_line": "",
    "error": "",
    "pni": {"valid": False},
    "accel": {"valid": False},
    "tick": None,
    "mcu_interval_ms": None,
    "rx_interval_ms": None,
    "packet_count": 0,
    "parse_error_count": 0,
}


def update_sensor(**kwargs):
    with sensor_lock:
        sensor_data.update(kwargs)


def sensor_snapshot():
    with sensor_lock:
        data = dict(sensor_data)
        last_valid = last_valid_rx_time

    fresh = last_valid is not None and time.monotonic() - last_valid <= SENSOR_TIMEOUT
    data["online"] = fresh
    if not fresh:
        data["pni"] = dict(data.get("pni") or {}, valid=False)
        data["accel"] = dict(data.get("accel") or {}, valid=False)
        if not data["error"]:
            data["error"] = "no recent valid STM32 packet"
    return data


def process_sensor_line(line):
    global last_valid_rx_time
    rx_time = time.monotonic()
    try:
        data = json.loads(line)
        if not isinstance(data, dict):
            raise ValueError("JSON root must be an object")
        if "temperature" not in data:
            raise ValueError("missing temperature field")
        temperature = data["temperature"]
        if temperature is not None:
            temperature = float(temperature)
            if not math.isfinite(temperature):
                raise ValueError("temperature is not finite")
        if not isinstance(data.get("pni"), dict) or not isinstance(data.get("accel"), dict):
            raise ValueError("missing pni or accel object")
        tick = data.get("tick")
        if not isinstance(tick, int) or isinstance(tick, bool):
            raise ValueError("invalid tick")

        with sensor_lock:
            previous_tick = sensor_data.get("tick")
            previous_rx = last_valid_rx_time
            data.update({
                "temperature": temperature,
                "unit": str(data.get("unit", "C")),
                "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                "online": True,
                "last_line": line,
                "error": "",
                "packet_count": sensor_data["packet_count"] + 1,
                "parse_error_count": sensor_data["parse_error_count"],
                "mcu_interval_ms": None if previous_tick is None else (tick - previous_tick) & 0xFFFFFFFF,
                "rx_interval_ms": None if previous_rx is None else round((rx_time - previous_rx) * 1000.0, 1),
            })
            sensor_data.update(data)
            last_valid_rx_time = rx_time
        return True
    except (ValueError, TypeError, KeyError) as exc:
        with sensor_lock:
            sensor_data.update(
                last_line=line[:2048],
                error=f"parse failed: {exc}",
                parse_error_count=sensor_data["parse_error_count"] + 1,
            )
        return False


def png_chunk(kind, payload):
    return (
        struct.pack(">I", len(payload)) + kind + payload
        + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
    )


def occupancy_grid_to_png(message):
    """Convert ROS bottom-left grid rows to top-left 8-bit grayscale PNG."""
    width = int(message.info.width)
    height = int(message.info.height)
    if width <= 0 or height <= 0 or width * height > 8_000_000:
        raise ValueError("invalid or excessive map dimensions")

    values = message.data
    raw = values.tobytes() if hasattr(values, "tobytes") else bytes(
        int(value) & 0xFF for value in values
    )
    if len(raw) != width * height:
        raise ValueError("map data size does not match dimensions")

    pixels = raw.translate(MAP_PALETTE)
    scanlines = b"".join(
        b"\x00" + pixels[row * width:(row + 1) * width]
        for row in range(height - 1, -1, -1)
    )
    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", header)
        + png_chunk(b"IDAT", zlib.compress(scanlines, level=3))
        + png_chunk(b"IEND", b"")
    )


def process_map_message(message):
    global map_image
    resolution = float(message.info.resolution)
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("invalid map resolution")

    png = occupancy_grid_to_png(message)
    pose = message.info.origin
    yaw = quaternion_yaw(pose.orientation)
    origin_x, origin_y = float(pose.position.x), float(pose.position.y)
    if not math.isfinite(origin_x) or not math.isfinite(origin_y):
        raise ValueError("non-finite map origin")
    with map_lock:
        version = map_state["version"] + 1
        map_image = png
        map_state.update({
            "available": True,
            "version": version,
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "frame_id": message.header.frame_id,
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": resolution,
            "origin": {
                "x": origin_x,
                "y": origin_y,
                "yaw": yaw,
            },
            "image_url": f"/map/image?v={version}",
            "error": "",
        })


def enqueue_map(message):
    try:
        map_queue.put_nowait(message)
    except queue.Full:
        try:
            map_queue.get_nowait()
            map_queue.task_done()
        except queue.Empty:
            pass
        map_queue.put_nowait(message)


def quaternion_yaw(q):
    values = [float(q.x), float(q.y), float(q.z), float(q.w)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite TF orientation")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError("invalid TF quaternion")
    x, y, z, w = [value / norm for value in values]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def invalidate_robot_pose(error):
    with robot_pose_lock:
        robot_pose_state.update(valid=False, error=str(error))


def process_robot_transform(transform, ros_now_seconds):
    """Cache base-frame pose in the target frame, rejecting old cached TF."""
    global robot_pose_rx_time
    stamp = transform.header.stamp
    stamp_seconds = int(stamp.sec) + int(stamp.nanosec) / 1e9
    age = float(ros_now_seconds) - stamp_seconds
    # A moving robot must have a timestamped dynamic chain. Zero stamps can
    # represent all-static TF and must not be accepted as live localisation.
    if stamp_seconds <= 0 or not math.isfinite(age) or age < -0.1 or age > ROBOT_POSE_TIMEOUT:
        raise ValueError("TF timestamp is missing, stale or in the future")
    translation = transform.transform.translation
    x, y, z = float(translation.x), float(translation.y), float(translation.z)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError("non-finite TF position")
    yaw = quaternion_yaw(transform.transform.rotation)
    with robot_pose_lock:
        robot_pose_rx_time = time.monotonic()
        robot_pose_state.update({
            "valid": True, "error": "", "frame_id": transform.header.frame_id,
            "child_frame_id": transform.child_frame_id,
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "tf_stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "tf_age_ms": round(max(age, 0) * 1000, 1),
            "x": x, "y": y, "z": z,
            "yaw": yaw, "yaw_deg": math.degrees(yaw),
        })


def project_robot_to_map(data, meta):
    """Coordinates on the full PNG, measured from its top-left outer edge."""
    if not meta["available"] or data["frame_id"] != meta["frame_id"]:
        return None
    origin = meta["origin"]
    dx, dy = data["x"] - origin["x"], data["y"] - origin["y"]
    cosine, sine = math.cos(origin["yaw"]), math.sin(origin["yaw"])
    grid_x = (cosine * dx + sine * dy) / meta["resolution"]
    grid_y = (-sine * dx + cosine * dy) / meta["resolution"]
    angle = -(data["yaw"] - origin["yaw"])
    return {
        "x": grid_x, "y": meta["height"] - grid_y,
        "yaw": math.atan2(math.sin(angle), math.cos(angle)),
        "in_bounds": 0 <= grid_x < meta["width"] and 0 <= grid_y < meta["height"],
        "width": meta["width"], "height": meta["height"],
        "map_version": meta["version"],
    }


def robot_pose_snapshot():
    with robot_pose_lock:
        data = dict(robot_pose_state)
        rx_time = robot_pose_rx_time
    elapsed = None if rx_time is None else max(0, time.monotonic() - rx_time)
    if data["valid"] and (elapsed is None or elapsed + data["tf_age_ms"] / 1000 > ROBOT_POSE_TIMEOUT):
        data.update(valid=False, error="no recent valid robot TF")
    data["pixel"] = None
    if data["valid"]:
        data["tf_age_ms"] = round(data["tf_age_ms"] + elapsed * 1000, 1)
        with map_lock:
            meta = dict(map_state)
        data["pixel"] = project_robot_to_map(data, meta)
    else:
        # Never let clients mistake cached coordinates for a live robot pose.
        for field in ("x", "y", "z", "yaw", "yaw_deg"):
            data[field] = None
    return data


def poll_robot_transform(buffer, node, latest_time, transform_exception):
    with map_lock:
        target_frame = map_state["frame_id"] or ROBOT_MAP_FRAME
    try:
        transform = buffer.lookup_transform(target_frame, ROBOT_BASE_FRAME, latest_time)
        process_robot_transform(transform, node.get_clock().now().nanoseconds / 1e9)
    except (transform_exception, ValueError) as exc:
        invalidate_robot_pose(exc)


def map_worker_loop():
    while True:
        message = map_queue.get()
        try:
            process_map_message(message)
        except Exception as exc:
            with map_lock:
                map_state["error"] = str(exc)
            print(f"Map conversion error: {exc}")
        finally:
            map_queue.task_done()


def update_recognition_image(image_bytes, content_type="image/jpeg"):
    global recognition_image

    with recognition_lock:
        recognition_image = image_bytes
        recognition_data.update(
            {
                "available": True,
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "size": len(image_bytes),
                "content_type": content_type,
                "error": "",
            }
        )


def get_camera():
    global camera

    with camera_lock:
        if camera is None or not camera.isOpened():
            camera = cv2.VideoCapture(CAMERA_INDEX)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

            if not camera.isOpened():
                camera.release()
                camera = None
                raise RuntimeError(f"camera {CAMERA_INDEX} open failed")

        return camera


def read_camera_jpeg():
    with camera_lock:
        cap = get_camera()
        ok, frame = cap.read()

    if not ok or frame is None:
        raise RuntimeError("camera read failed")

    ok, jpg = cv2.imencode(".jpg", frame)
    if not ok:
        raise RuntimeError("jpg encode failed")

    return jpg.tobytes()


def generate_camera_frames():
    delay = 1.0 / max(CAMERA_FPS, 1)

    while True:
        try:
            jpg = read_camera_jpeg()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            )
        except Exception as exc:
            print(f"Camera stream error: {exc}")
            time.sleep(1)
            continue

        time.sleep(delay)


def serial_read_loop():
    import serial

    while True:
        try:
            print(f"Opening serial port {SERIAL_PORT} @ {BAUD_RATE}")
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT) as ser:
                ser.reset_input_buffer()
                print("Serial port opened")
                framer = JsonLineFramer()
                last_log_time = time.monotonic()

                while True:
                    for line in framer.feed(ser.read(256)):
                        process_sensor_line(line)
                    if time.monotonic() - last_log_time >= SERIAL_LOG_INTERVAL:
                        last_log_time = time.monotonic()
                        snapshot = sensor_snapshot()
                        print(
                            f"STM32 packets={snapshot['packet_count']}, "
                            f"online={snapshot['online']}, parse_errors={snapshot['parse_error_count']}, "
                            f"framing_errors={framer.dropped_lines}"
                        )

        except Exception as exc:
            update_sensor(online=False, error=str(exc))
            print(f"Serial error: {exc}")
            time.sleep(2)


def ros_read_loop():
    while True:
        node = None
        listener = None
        rclpy = None
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.signals import SignalHandlerOptions
            from nav_msgs.msg import OccupancyGrid
            from std_msgs.msg import String
            from rclpy.time import Time
            from tf2_ros import TransformException
            from tf2_ros.buffer import Buffer
            from tf2_ros.transform_listener import TransformListener

            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = Node("stm32_flask_bridge")
            node.create_subscription(
                String, "/stm32/sensor_json", lambda msg: process_sensor_line(msg.data), 10
            )
            # Volatile depth-1 QoS accepts the next live /map update without
            # requiring a latched publisher and does not backlog old maps.
            node.create_subscription(OccupancyGrid, "/map", enqueue_map, 1)
            buffer = Buffer(node=node)
            listener = TransformListener(buffer, node)
            node.create_timer(
                1.0 / ROBOT_POSE_RATE,
                lambda: poll_robot_transform(buffer, node, Time(), TransformException),
            )
            rclpy.spin(node)
        except Exception as exc:
            update_sensor(online=False, error=f"ROS2 subscription error: {exc}")
            invalidate_robot_pose(f"ROS2 subscription error: {exc}")
            print(f"ROS2 subscription error: {exc}")
        finally:
            if listener is not None:
                listener.unregister()
            if node is not None:
                node.destroy_node()
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
        time.sleep(2)


@app.route("/")
def index():
    state = sensor_snapshot()
    online = "在线" if state["online"] else "离线"
    return Response(
        "<!doctype html><html lang='zh-CN'><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>STM32 数据服务</title>"
        "<style>body{font:16px/1.6 sans-serif;max-width:760px;margin:40px auto;"
        "padding:0 20px}li{margin:10px 0}code{background:#f2f2f2;padding:2px 5px}</style>"
        "<h1>STM32 数据服务</h1>"
        f"<p>当前数据源：<code>{DATA_SOURCE}</code>；最新数据状态：<strong>{online}</strong>。</p>"
        "<p>当前提供 9 个业务接口、1 个调试接口：</p>"
        "<ul><li><a href='/sensor'>/sensor</a>：温度、PNI 姿态和加速度数据</li>"
        "<li><a href='/map/meta'>/map/meta</a>：地图尺寸、分辨率和原点</li>"
        "<li><a href='/map/image'>/map/image</a>：实时地图 PNG 图片</li>"
        "<li><a href='/robot/pose'>/robot/pose</a>：地图中的小车位置、朝向及图片坐标</li>"
        "<li><a href='/snapshot'>/snapshot</a>：摄像头单帧图片</li>"
        "<li><a href='/video_feed'>/video_feed</a>：摄像头视频流</li>"
        "<li><code>POST /recognition/upload</code>：上传识别图片</li>"
        "<li><a href='/recognition/image'>/recognition/image</a>：最新识别图片</li>"
        "<li><a href='/recognition/status'>/recognition/status</a>：识别图片状态</li>"
        "<li><a href='/debug/serial'>/debug/serial</a>：接收调试信息</li></ul>"
        "<p>图片尚未上传时 <code>/recognition/image</code> 会返回 404；"
        "摄像头未就绪时图片接口可能报错。</p>"
        "<p>小车位姿来自地图 TF；未收到或超过时效时 /robot/pose 返回 503，"
        "此时不要绘制旧位置。保存 RViz 显示配置不会保存地图。</p>"
        "<p>PNI 的 <code>valid=true</code> 表示本次数据格式和时效有效；"
        "<code>status=0</code> 仍表示姿态可信度不足，需要结合校准状态判断。</p>"
        "<p><code>0.0.0.0</code> 是服务监听地址；从其他设备访问时请使用"
        " <code>rspi.local</code> 或树莓派的实际 IP。</p>"
        "</html>",
        mimetype="text/html",
    )


@app.route("/sensor")
def get_sensor():
    return jsonify({"success": True, "data": sensor_snapshot()})


@app.route("/map/meta")
def get_map_meta():
    with map_lock:
        data = dict(map_state)
    return jsonify({"success": data["available"], "data": data}), (
        200 if data["available"] else 503
    )


@app.route("/map/image")
def get_map_image():
    with map_lock:
        png = map_image
    if png is None:
        return jsonify({"success": False, "message": "map not received yet"}), 503
    response = Response(png, mimetype="image/png")
    response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/debug/serial")
def debug_serial():
    return jsonify({"success": True, "data": sensor_snapshot()})


@app.route("/robot/pose")
def get_robot_pose():
    data = robot_pose_snapshot()
    return (
        jsonify({"success": data["valid"], "data": data}),
        200 if data["valid"] else 503,
        {"Cache-Control": "no-store"},
    )


@app.route("/snapshot")
def snapshot():
    try:
        jpg = read_camera_jpeg()
        return Response(jpg, mimetype="image/jpeg")
    except Exception as exc:
        return jsonify({"success": False, "message": str(exc)}), 500


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_camera_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/recognition/upload", methods=["POST"])
def upload_recognition_image():
    image_bytes = None
    content_type = "image/jpeg"

    if "image" in request.files:
        image_file = request.files["image"]
        image_bytes = image_file.read()
        content_type = image_file.mimetype or content_type
    else:
        image_bytes = request.get_data()
        content_type = request.content_type or content_type

    if not image_bytes:
        return jsonify({"success": False, "message": "missing image"}), 400

    update_recognition_image(image_bytes, content_type)
    return jsonify({"success": True, "data": dict(recognition_data)})


@app.route("/recognition/image")
def get_recognition_image():
    with recognition_lock:
        image_bytes = recognition_image
        content_type = recognition_data["content_type"]

    if image_bytes is None:
        return jsonify({"success": False, "message": "no recognition image"}), 404

    return Response(image_bytes, mimetype=content_type)


@app.route("/recognition/status")
def get_recognition_status():
    with recognition_lock:
        data = dict(recognition_data)

    return jsonify({"success": True, "data": data})


if __name__ == "__main__":
    if DATA_SOURCE not in ("ros", "serial"):
        raise SystemExit("STM32_DATA_SOURCE must be 'ros' or 'serial'")
    receiver = ros_read_loop if DATA_SOURCE == "ros" else serial_read_loop
    if DATA_SOURCE == "ros":
        threading.Thread(target=map_worker_loop, daemon=True).start()
    threading.Thread(target=receiver, daemon=True).start()

    browser_base = f"http://{PUBLIC_HOST}:{SERVICE_PORT}"
    print(f"STM32 data service started (source: {DATA_SOURCE})")
    print(f"Browser home: {browser_base}/")
    print(f"Sensor API: {browser_base}/sensor")
    print(f"Map metadata API: {browser_base}/map/meta")
    print(f"Map image API: {browser_base}/map/image")
    print(f"Robot pose API: {browser_base}/robot/pose")
    print(f"Snapshot API: {browser_base}/snapshot")
    print(f"Video API: {browser_base}/video_feed")
    print(f"Recognition upload (POST): {browser_base}/recognition/upload")
    print(f"Recognition image: {browser_base}/recognition/image")
    print(f"Recognition status: {browser_base}/recognition/status")
    print(f"Receive debug: {browser_base}/debug/serial")
    print("0.0.0.0 is the listen address, not a browser URL.")
    app.run(host="0.0.0.0", port=SERVICE_PORT, threaded=True, debug=False)
