#!/usr/bin/env bash
# Raspberry Pi: RPLIDAR -> RF2O -> SLAM Toolbox. Run RViz in the VM separately.

set -eo pipefail

ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
STM32_PORT="${STM32_SERIAL_PORT:-/dev/ttyUSB0}"
LIDAR_PORT="${RPLIDAR_SERIAL_PORT:-/dev/ttyUSB1}"
SLAM_PARAMS_FILE="${SLAM_PARAMS_FILE:-$ROS2_WS/slam_params.yaml}"
PNI_NODE_FILE="$ROS2_WS/stm32_pni_ros2_node.py"

for required_file in /opt/ros/jazzy/setup.bash "$ROS2_WS/install/setup.bash" \
    "$PNI_NODE_FILE" "$SLAM_PARAMS_FILE"; do
    if [[ ! -f "$required_file" ]]; then
        echo "错误：缺少文件 $required_file" >&2
        exit 1
    fi
done

# ROS setup scripts may reference unset variables; do not enable nounset here.
source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"

for port in "$STM32_PORT" "$LIDAR_PORT"; do
    if [[ ! -e "$port" ]]; then
        echo "错误：找不到串口 $port。请检查 USB 连接及端口对应关系。" >&2
        exit 1
    fi
done

if [[ "$STM32_PORT" == "$LIDAR_PORT" ]]; then
    echo "错误：STM32 与雷达不能使用同一个串口。" >&2
    exit 1
fi

for package in rplidar_ros rf2o_laser_odometry slam_toolbox tf2_ros; do
    if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
        echo "错误：当前 ROS2 环境中找不到软件包 $package" >&2
        exit 1
    fi
done

PIDS=()
cleanup() {
    trap - EXIT INT TERM
    echo "正在停止建图节点..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "启动 PNI 节点（不发布里程计和 TF）..."
STM32_SERIAL_PORT="$STM32_PORT" \
STM32_BAUD_RATE=115200 \
PNI_ENABLE_ACCEL_POSITION=0 \
PNI_PUBLISH_ODOM=0 \
PNI_PUBLISH_TF=0 \
python3 "$PNI_NODE_FILE" &
PIDS+=("$!")

echo "启动思岚雷达：$LIDAR_PORT"
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p "serial_port:=$LIDAR_PORT" \
    -p serial_baudrate:=460800 \
    -p frame_id:=laser \
    -p angle_compensate:=true \
    -p scan_mode:=Standard &
PIDS+=("$!")

echo "发布 base_link -> laser 静态 TF..."
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link \
    --child-frame-id laser &
PIDS+=("$!")

sleep 3

echo "启动 RF2O（/scan -> /odom_rf2o，发布 odom -> base_link）..."
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py &
PIDS+=("$!")

sleep 3

echo "启动 SLAM Toolbox：$SLAM_PARAMS_FILE"
ros2 launch slam_toolbox online_async_launch.py \
    "slam_params_file:=$SLAM_PARAMS_FILE" \
    use_sim_time:=false &
PIDS+=("$!")

echo "建图节点已启动。请在虚拟机运行 RViz；按 Ctrl+C 停止本脚本。"
if wait -n "${PIDS[@]}"; then
    echo "一个建图进程已退出，正在停止其余进程。" >&2
else
    echo "一个建图进程异常退出，正在停止其余进程。" >&2
fi
exit 1
