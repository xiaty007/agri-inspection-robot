# AGENT_HANDOFF

> **2026-10-08 交接入口更新：先阅读 [最新项目交接文档](PROJECT_HANDOFF_20261008.md)。**
>
> 用户已确认 PNI 连续接收恢复、STM32 稳定性测试大致完成，且雷达已可以建图。最新文档记录最终 FIFO 流解析、备份与固件、仍开启的测试输出以及树莓派端待修复事项。
>
> **下面正文为 2026-08-31 历史记录，不代表当前实现。** 其中 FIFO、PNI 异常处理、构建状态及建图进度须以最新文档和实际代码为准。

请下一个对话先读取本文件。当前项目是 STM32F407 + 树莓派 + PNI 惯性/姿态数据 + 思岚 C1M1 2D 雷达 + ROS2/RViz/Flask/摄像头上传的综合调试项目。

## 1. 工作区与重要约束

- Windows 工作区：`C:\Users\33615\Documents\New project111`
- 当前 Keil/STM32 工程主目录：`407_gps初始化失败 - 副本`
- 已保存过一个早期整包备份：`v1`
- 根目录里已有说明文档：
  - `项目对话迁移摘要.md`
  - `项目使用说明书.md`
  - `项目使用说明书.docx`
- 仓库规则：禁止批量删除文件或目录。需要删除时只能一次删除一个明确路径的文件；如果必须批量清理，先停下让用户手动处理。
- 工作区里有很多 `.bak_before_*` 备份文件，不要随意删。

## 2. 当前硬件状态

### STM32 与树莓派通信

- 用户发现树莓派 GPIO RX/TX 引脚有烧黑痕迹，树莓派板载 GPIO 串口不建议继续作为主通信口。
- STM32 串口一经测试仍可正常收发。
- 用户只有一个 USB 转 TTL，目前建议 STM32 到树莓派采用 USB-TTL 单向传输。
- 当前建议接线：
  - STM32 USART2_TX / PA2 -> USB-TTL RX
  - STM32 GND -> USB-TTL GND
  - 不需要接 STM32 RX，如果只做 STM32 向树莓派发送 JSON。
- 树莓派上 USB-TTL 通常显示为 `/dev/ttyUSB0`，思岚雷达可能是 `/dev/ttyUSB1`。重启或重插后编号可能互换，需要用 `ls /dev/ttyUSB*` 和 `dmesg | tail` 确认。

### 传感器

- DS18B20 温度传感器已经能正常读数，曾出现 85 度问题，后来通过 main 里先初始化并保留用户的推挽 GPIO 模式解决。
- PNI 使用 I2C，不占用 USART。当前已经能输出姿态角和加速度，但偶尔会出现明显异常包。
- 思岚 C1M1 2D 雷达通过 USB 接树莓派，ROS2 中可以看到 `/scan`，用户曾在 RViz 里看到点云。

## 3. STM32 当前代码状态

主要文件：

- `407_gps初始化失败 - 副本\Core\Src\main.c`
- `407_gps初始化失败 - 副本\Core\Src\sensors.c`
- `407_gps初始化失败 - 副本\Core\Inc\sensors.h`
- `407_gps初始化失败 - 副本\Core\Src\pni.c`
- `407_gps初始化失败 - 副本\Core\Inc\pni.h`
- `407_gps初始化失败 - 副本\Core\Src\usart.c`

当前设计方向：

- `main.c` 只负责初始化和循环调度，不把具体实现堆在 main 里。
- `main.c` 当前核心逻辑大致是：

```c
HAL_TIM_Base_Start(&htim4);
DS18B20_ReportStartup();
/* GPS_Init(); */
PNI_ReportStartup();

while (1)
{
    Sensor_UploadTask();
    PNI_UpdateTask();
    DS18B20_UpdateTask();
    /* GPS_Parse(); */
}
```

- `Sensor_UploadTask()` 独立负责按固定频率发 JSON，不再跟温度更新绑定。
- `DS18B20_UpdateTask()` 独立更新温度缓存，避免温度转换 delay 卡住串口上传。
- 传输到树莓派的 JSON 通过 USART2 发送，当前目标周期约 50 ms：

```c
#define SENSOR_UPLOAD_INTERVAL_MS 50U
HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, (uint16_t)len, 30);
```

- `printf` 已经重定向回 USART1，避免调试文本混进 USART2 JSON：

```c
int fputc(int ch, FILE *stream)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
```

- USART 现状大致：
  - USART1：115200，调试/电脑串口
  - USART2：115200，STM32 -> 树莓派 JSON
  - USART3：38400，GPS 相关

PNI 当前状态：

- `PNI_UPDATE_POLL_MS` 曾调整到 100 ms。
- `PNI_UpdateTask()` 已改成每轮从 FIFO 多读几段，减少只读一次导致数据不更新的问题。
- PNI 初始化里同时启用了 Accelerometer 和 Orientation。
- 曾做过单独 Z 轴加速度快速测试，后来按用户要求把测试函数和冗余逻辑删回去了。

## 4. 当前 STM32 发出的 JSON 格式

正常包类似：

```json
{"tick":37589,"temperature":29.44,"unit":"C","pni":{"valid":true,"heading":-144.92,"pitch":1.01,"roll":3.14,"status":0},"accel":{"valid":true,"x":0.553,"y":-0.226,"z":10.156,"status":3}}
```

异常包示例：

```json
{"tick":37670,"temperature":29.44,"unit":"C","pni":{"valid":true,"heading":-191.29,"pitch":4.36,"roll":4.45,"status":141},"accel":{"valid":true,"x":-1.225,"y":-36.486,"z":36.751,"status":1}}
```

当前重点问题：

- 用户不是说慢漂，而是 PNI/accel 会偶发大幅异常跳变。
- 典型异常特征：heading 超出 `[-180, 180]`，PNI status 变成 141，加速度 Y/Z 出现几十 m/s^2 级别异常。
- Python 端已经加了异常包过滤，但这只是先挡住 ROS/RViz 显示跳变；根因可能还在 STM32 端 PNI FIFO 解析、数据对齐、status 判断或 I2C/FIFO 读包完整性。

## 5. 树莓派 Flask 文件状态

主要文件：

- `raspberry_pi_temp_serial_service.py`
- `raspberry_pi_temp_serial_service_v1.py`
- 多个 `raspberry_pi_temp_serial_service.py.bak_before_*`

功能：

- 从 STM32 串口读取 JSON。
- Flask 提供 `/sensor`、`/snapshot`、`/video_feed` 等接口。
- 摄像头画面、YOLO 识别结果图片上传也做过改动。

注意：

- 如果使用 USB-TTL，运行 Flask 时需要指定：

```bash
STM32_SERIAL_PORT=/dev/ttyUSB0 python3 raspberry_pi_temp_serial_service.py
```

- Flask 串口服务和 ROS2 PNI 节点不能同时打开同一个 `/dev/ttyUSB0`，否则会抢串口。

## 6. ROS2 / RViz / 雷达当前状态

树莓派系统：

- Ubuntu 24
- ROS2 Jazzy
- 用户名：`rsp`
- 树莓派 IP：`192.168.137.23`
- ROS 工作区大致为：`~/ros2_ws`

雷达：

- 设备：思岚 / SLAMTEC C1M1，用户确认是 `rp` 系列。
- 之前 `rplidar_ros` launch 文件报过 `rplidar_node` not found，后来用户说重新编译后能正常执行。
- 直接启动建议：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB1 \
  -p serial_baudrate:=460800 \
  -p frame_id:=laser \
  -p angle_compensate:=true \
  -p scan_mode:=Standard
```

检查雷达：

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

RViz：

- 已安装并打开过。
- 如果只看雷达，Fixed Frame 可以设 `laser`。
- 如果同时看 PNI、路径和雷达，Fixed Frame 建议设 `odom`。
- 需要静态 TF：

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
```

不要再额外发布静态 `odom -> base_link`，因为当前 Python PNI 节点已经动态发布 `odom -> base_link`。

## 7. ROS2 PNI 节点状态

主要文件：

- `stm32_pni_ros2_node.py`
- 备份：`stm32_pni_ros2_node.py.bak_before_*`

作用：

- 从 STM32 JSON 串口读取 PNI 姿态角和加速度。
- 发布 ROS2 话题：
  - `/pni/heading`
  - `/pni/rpy`
  - `/pni/imu`
  - `/pni/odom`
  - `/pni/path`
  - `/stm32/sensor_json`
  - `/pni/status`
  - `/pni/heading_marker`
- 动态发布 TF：`odom -> base_link`
- 默认不开启加速度积分位置，因为无编码器、只靠加速度积分会漂得很快。

当前关键参数：

```python
SERIAL_PORT = os.environ.get("STM32_SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("STM32_BAUD_RATE", "115200"))
FRAME_ID = os.environ.get("PNI_FRAME_ID", "base_link")
ODOM_FRAME_ID = os.environ.get("PNI_ODOM_FRAME_ID", "odom")
ENABLE_ACCEL_POSITION = os.environ.get("PNI_ENABLE_ACCEL_POSITION", "0") == "1"
MAX_HEADING_JUMP_DEG = float(os.environ.get("PNI_MAX_HEADING_JUMP_DEG", "45.0"))
MAX_ACCEL_ABS = float(os.environ.get("PNI_MAX_ACCEL_ABS", "20.0"))
MAX_ACCEL_NORM = float(os.environ.get("PNI_MAX_ACCEL_NORM", "20.0"))
```

最新改动：

- 加了 `validate_pni_packet()`：
  - 拒绝 `pni.valid == false`
  - 拒绝 `status >= 128`
  - 拒绝 heading/pitch/roll 非有限数
  - 拒绝 heading 超出 `[-180, 180]`
  - 拒绝 pitch/roll 超出 `[-90, 90]`
  - 拒绝单包 heading 跳变超过 `MAX_HEADING_JUMP_DEG`
- 加了 `validate_accel_packet()`：
  - 拒绝 `accel.valid == false`
  - 拒绝 x/y/z 非有限数
  - 拒绝任一轴绝对值超过 `MAX_ACCEL_ABS`
  - 拒绝三轴模长超过 `MAX_ACCEL_NORM`
- `/pni/status` 里有 `rejected_packets` 和 `last_reject_reason`，用来观察过滤是否生效。

建议下一步先做：

```powershell
python -m py_compile "C:\Users\33615\Documents\New project111\stm32_pni_ros2_node.py"
```

然后发到树莓派：

```powershell
scp "C:\Users\33615\Documents\New project111\stm32_pni_ros2_node.py" rsp@192.168.137.23:~/ros2_ws/stm32_pni_ros2_node.py
```

树莓派运行：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
STM32_SERIAL_PORT=/dev/ttyUSB0 STM32_BAUD_RATE=115200 python3 stm32_pni_ros2_node.py
```

观察：

```bash
ros2 topic echo /pni/status
ros2 topic echo /stm32/sensor_json
ros2 topic hz /pni/odom
ros2 topic echo /pni/heading
```

## 8. 当前最值得继续处理的问题

### A. PNI 异常跳变

用户当前最关心的是 PNI 数据偶发大幅跳变，例如 heading 突然到 -191，status 到 141，加速度突然到几十。

短期方案：

- 先验证 `stm32_pni_ros2_node.py` 里的过滤是否能让 RViz 箭头不再突然乱跳。
- 用 `/pni/status` 看 `rejected_packets` 是否增加，`last_reject_reason` 是否符合异常包。

中期方案：

- 如果 `/stm32/sensor_json` 里仍能看到大量异常包，要回到 STM32 侧解决：
  - PNI FIFO 是否按完整 8 字节包解析；
  - 是否读到半包或错位包；
  - 是否每次 FIFO 读完后缓存位置正确移动；
  - 是否应该在 STM32 端就丢弃 `status >= 128` 或角度越界包；
  - 是否需要在 `pni.c` 保留 last-good packet，不把坏包交给 `sensors.c` 上传。

### B. 无编码器惯导建图限制

- 用户想实现惯导，但小车没有编码器。
- 只有 PNI 姿态 + 加速度时，短时间姿态可以用，位置积分很容易漂。
- 真实建图建议优先用雷达 SLAM，PNI 作为 yaw/IMU 辅助，不要直接用加速度积分当可靠里程计。
- 如果后续要继续做 ROS 建图，可以考虑：
  - `slam_toolbox` 用 `/scan` 建图；
  - PNI 发布 `/imu` 或 yaw TF 辅助；
  - 后续若能加轮速编码器，再融合 IMU + 编码器。

### C. RViz 显示与 TF

如果 RViz 没东西：

1. 确认 `/scan` 有数据。
2. 确认 `/pni/odom`、`/pni/path`、`/pni/heading_marker` 有数据。
3. 确认 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

4. Fixed Frame 设 `odom`。
5. 如果只看雷达，Fixed Frame 临时设 `laser` 排除 TF 问题。

## 9. 常用命令清单

Windows 发文件到树莓派：

```powershell
scp "C:\Users\33615\Documents\New project111\stm32_pni_ros2_node.py" rsp@192.168.137.23:~/ros2_ws/stm32_pni_ros2_node.py
scp "C:\Users\33615\Documents\New project111\raspberry_pi_temp_serial_service.py" rsp@192.168.137.23:~/raspberry_pi_temp_serial_service.py
```

树莓派看 USB 设备：

```bash
ls /dev/ttyUSB*
dmesg | tail -30
```

树莓派直接看 STM32 串口：

```bash
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
```

退出 miniterm：

```text
Ctrl+]
```

ROS2 环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

启动 RViz：

```bash
rviz2
```

## 10. 接手建议

接下来的最佳顺序：

1. 先确认本地 `stm32_pni_ros2_node.py` 能通过 `python -m py_compile`。
2. 把最新 Python 节点 scp 到树莓派。
3. 只运行 STM32 PNI ROS2 节点，不运行 Flask，避免抢 `/dev/ttyUSB0`。
4. 用 `/pni/status` 验证异常包过滤。
5. 如果过滤有效，再打开雷达和 RViz 看箭头是否稳定。
6. 如果原始 `/stm32/sensor_json` 仍有大量异常包，再回 STM32 的 `pni.c` 做 FIFO 对齐和 last-good 过滤。

## 11. 最新建图程序与方案总结（2026-10-08）

用户已经分别测试过两套基于思岚 C1M1 二维雷达的 ROS2 建图方案。树莓派为 Raspberry Pi 5B 8GB，Ubuntu 24.04，ROS2 Jazzy。

### 11.1 硬件端口和基础话题

- STM32 / PNI：`/dev/ttyUSB0`
- 思岚 C1M1 雷达：`/dev/ttyUSB1`
- 雷达驱动包：`rplidar_ros`
- 雷达输出话题：`/scan`
- 雷达坐标系：`laser`
- 车体坐标系：`base_link`
- 雷达固定安装关系：`base_link -> laser`

雷达启动参考命令：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB1 \
  -p serial_baudrate:=460800 \
  -p frame_id:=laser \
  -p angle_compensate:=true \
  -p scan_mode:=Standard
```

如果雷达启动文件没有发布固定 TF，则单独运行：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id laser
```

### 11.2 方案A：RF2O + SLAM Toolbox（目前推荐作为后续主方案）

处理结构：

```text
/scan -> RF2O -> odom -> base_link
/scan + 雷达里程计 -> SLAM Toolbox -> map 和 map -> odom
```

使用程序：

- `rplidar_ros`：发布 `/scan`
- `rf2o_laser_odometry`：用连续二维雷达帧计算雷达里程计
- `slam_toolbox`：扫描匹配、位姿图建图和回环优化
- `tf2_ros static_transform_publisher`：发布 `base_link -> laser`
- `rviz2`：显示 `/map`、`/scan` 和 TF

RF2O 已经从 `MAPIRlab/rf2o_laser_odometry` 的 ROS2 分支编译完成；用户确认 `/odom_rf2o` 有数据，小车移动时 x/y 会变化。

SLAM Toolbox 当前关键参数：

```yaml
base_frame: base_link
odom_frame: odom
map_frame: map
use_scan_matching: true
minimum_travel_distance: 0.05
minimum_travel_heading: 0.087
map_update_interval: 1.0
enable_interactive_mode: false
```

该方案实测地图墙体轮廓比较清晰、重影较少。它也是当前建议继续接入轮速编码器、`robot_localization` 和 Nav2 的主方案。

运行该方案时不要启动 PNI 节点发布的 `odom -> base_link`，也不要再发布静态 `odom -> base_link`，否则会出现 TF 冲突。

### 11.3 方案B：Cartographer 2D 纯雷达建图（保留为对照方案）

已经安装：

```text
ros-jazzy-cartographer 2.0.9004
ros-jazzy-cartographer-ros 2.0.9003
```

处理结构：

```text
/scan -> Cartographer 2D -> 位姿、子地图、回环和动态 TF
Cartographer submaps -> cartographer_occupancy_grid_node -> /map
```

纯雷达配置文件位于树莓派：

```text
/home/rsp/ros2_ws/cartographer_config/c1m1_2d.lua
```

配置的核心含义：

```lua
map_frame = "map"
tracking_frame = "base_link"
published_frame = "base_link"
odom_frame = "odom"
provide_odom_frame = true
use_odometry = false
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
```

启动 Cartographer：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run cartographer_ros cartographer_node \
  -configuration_directory /home/rsp/ros2_ws/cartographer_config \
  -configuration_basename c1m1_2d.lua \
  --ros-args -p use_sim_time:=false
```

发布占据栅格地图：

```bash
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.05 \
  -publish_period_sec 0.5
```

Cartographer 测试地图覆盖范围较大，但墙体出现较厚的灰黑重影，当前稳定性不如 RF2O + SLAM Toolbox。它适合继续作为纯二维雷达、不依赖外部里程计的对照测试。

运行 Cartographer 时必须停止 RF2O、SLAM Toolbox、PNI 动态 TF和任何静态 `odom -> base_link`。

### 11.4 RViz 和 TF 检查

RViz 建图时：

- `Fixed Frame` 设置为 `map`
- 添加 `Map`，话题 `/map`
- 添加 `LaserScan`，话题 `/scan`
- 添加 `TF`

正常 TF 应为：

```text
map -> odom -> base_link -> laser
```

必须保证每一条动态 TF 只有一个发布者。

### 11.5 当前结论和后续路线

- 正式后续开发优先使用：RF2O/轮速里程计 + SLAM Toolbox。
- Cartographer 2D 保留为雷达和 TF 的独立验证工具。
- PNI 加速度积分位置漂移太大，不再作为主里程计。
- 后续接入左右轮编码器，让编码器提供平移和短期里程计。
- PNI 后续只提供姿态、航向或角速度，不直接发布 `odom -> base_link`。
- 最终用 `robot_localization` 融合轮速编码器和 PNI，再把融合后的 `odom -> base_link` 提供给 SLAM Toolbox和 Nav2。
- 当前 PNI 偶发炸点最可能来自 STM32 的 FIFO 错位解析；Python 过滤只是下游保护，根因仍应在 `pni.c` 修复。

