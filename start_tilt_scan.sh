#!/usr/bin/env bash
# Separate experiment: do NOT run start_mapping_rf2o.sh at the same time.
set -eo pipefail
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
STM32_PORT="${STM32_SERIAL_PORT:-/dev/ttyUSB0}"
LIDAR_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB1}"
for file in /opt/ros/jazzy/setup.bash "$ROS2_WS/install/setup.bash" \
    "$ROS2_WS/stm32_pni_ros2_node.py" "$ROS2_WS/stm32_serial_protocol.py" \
    "$ROS2_WS/pni_tilt_cloud_core.py" "$ROS2_WS/pni_tilt_cloud_preview.py"; do
    if [[ ! -f "$file" ]]; then echo "缺少文件：$file" >&2; exit 1; fi
done
for port in "$STM32_PORT" "$LIDAR_PORT"; do
    if [[ ! -e "$port" ]]; then echo "找不到串口：$port" >&2; exit 1; fi
done
if [[ "$STM32_PORT" == "$LIDAR_PORT" ]]; then
    echo "两个设备不能使用同一串口" >&2; exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"
ros2 pkg prefix rplidar_ros >/dev/null
python3 -c 'import serial, rclpy; from sensor_msgs.msg import PointCloud2; from std_srvs.srv import Trigger'
PIDS=()
cleanup() {
    trap - EXIT INT TERM
    echo "停止本次点云实验的三个进程..."
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "请确认旧建图、旧串口节点和 yaw 融合已停止；本脚本不强制结束其他程序。"
STM32_SERIAL_PORT="$STM32_PORT" STM32_BAUD_RATE=115200 \
    PNI_ENABLE_ACCEL_POSITION=0 PNI_PUBLISH_ODOM=0 PNI_PUBLISH_TF=0 \
    PNI_PUBLISH_MARKER=0 python3 "$ROS2_WS/stm32_pni_ros2_node.py" &
PIDS+=("$!")
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p "serial_port:=$LIDAR_PORT" -p serial_baudrate:=460800 \
    -p frame_id:=laser -p angle_compensate:=true -p scan_mode:=Standard &
PIDS+=("$!")
python3 "$ROS2_WS/pni_tilt_cloud_preview.py" "$@" &
PIDS+=("$!")
echo "实验已启动，不运行 RF2O/SLAM，不发布 TF。RViz Fixed Frame: scan_reference"
echo "只抬头低头、停稳采集。Ctrl+C 停止本次实验。"
if wait -n "${PIDS[@]}"; then
    echo "一个进程已退出，停止本次实验" >&2
else
    echo "一个进程异常退出，停止本次实验" >&2
fi
exit 1
