# 手持倾斜点云试验（2026-10-14）

先看三维点云展开效果，**尚未生成2.5D高度栅格**。不修改STM32固件、原建图脚本、
Flask或TF；不运行yaw融合。新增程序独立订阅`/scan`和`/stm32/sensor_json`。

## 假设与边界

- 雷达与PNI固定在同一硬板上，相对位置/方向不变。尽量保持雷达扫描中心原地不动。
- 没有测量手持平移，不使用加速度积分。手移动产生的重影不能由此程序消除。
- 默认雷达与PNI坐标轴一致，右手坐标，X向前、Y向左、Z向上。
  默认安装旋转为0，不代表实物外参已确认。发现方向不对需核对安装和PNI坐标约定。
- 默认仅使用pitch/roll，忽略heading。不左右转身，不用该模式扫描四周拼接。
- 参考系`scan_reference`的原点是假设固定的雷达中心，Z为向上方向。
  默认`sensor_height=0`，高度相对雷达中心，不是离地高度。
- 保留用户实验假设：status=0允许，但valid、角度范围、更新时间仍检查。
- 初始姿态不会被整体归零：pitch/roll用于重力方向；可选heading只减初始航向零点。

## 算法

距离点`(r*cos(theta), r*sin(theta), 0)`经过雷达到PNI的安装旋转，再经过
`Rz(yaw)*Ry(pitch)*Rx(roll)`旋转。手持实验默认yaw=0。
转换后每个3cm体素保留最新观测，最多50000个体素，最远8m。
2026-10-14发布频率由2Hz提高到默认10Hz，`publish_rate_hz`可设1～30Hz；
状态消息仍约2Hz。提高点云发布频率不增加雷达测量频率，也不绕过稳定性检查。

PNI时间近似为ROS接收时间减MCU缓存年龄，扫描时间使用LaserScan首束时间戳、
time_increment；缺少time_increment时用scan_time保守估计整帧时长。
缓冲扫描等待姿态到达，要求时间接近、连续约0.6秒姿态角速度低于5度/秒。
检测移动时不累积；没有逐束运动去畸变。因此每次倾斜后停稳至少1秒再采集。
该判断只检测旋转，无法判断平移；握住不动仍是实验要求。
MCU重启清空旧点云以避免混入不同零点；输入停止时保留旧点云并报告停止采集。

## 部署

先在原一键建图终端Ctrl+C，停止RF2O/SLAM和旧串口节点；停止旧yaw融合。
如需保留当前二维地图，应先用原流程保存，本实验不会保存SLAM地图。
不要同时运行两个串口采集者，也不要直接给原二维SLAM输入倾斜扫描。
本脚本只停止它自己启动的进程，不强制结束其他程序。

Windows PowerShell上传（在项目目录执行）：

```powershell
cd "C:\Users\33615\Documents\New project111"
scp .\pni_tilt_cloud_core.py .\pni_tilt_cloud_preview.py .\start_tilt_scan.sh rsp@rspi.local:/home/rsp/ros2_ws/
```

树莓派上需已有`~/ros2_ws/stm32_pni_ros2_node.py`和`stm32_serial_protocol.py`。
缺失时从同一工作区上传，不能运行历史抢串口或重复TF版本。

树莓派SSH终端启动，不修改现有ROS_DOMAIN_ID：

```bash
sed -i 's/\r$//' ~/ros2_ws/start_tilt_scan.sh
bash ~/ros2_ws/start_tilt_scan.sh
```

该脚本启动三个进程：STM32串口节点、雷达驱动、点云节点。
默认STM32 USB0、雷达USB1，115200/460800；设备身份需与当前接线一致。
没有任何动态或静态TF发布，不运行RF2O/SLAM，也不自动启动RViz。
Ctrl+C同时停止本次三个进程。原建图恢复仍用原`start_mapping_rf2o.sh`。

## RViz

可继续使用已连通的虚拟机本地RViz；网络/ROS域设置沿用原来已工作的配置。
如用X11，在Windows/虚拟机`ssh -Y rsp@rspi.local`后，Pi新终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

- Fixed Frame手动输入`scan_reference`，不需要TF，因为点云已在此参考系中。
- 取消旧Map/TF/MarkerArray显示，添加PointCloud2，Topic`/pni_scan_cloud`。
- PointCloud2的Reliability选Reliable（若已选Best Effort也兼容），Durability选Volatile。
- Color Transformer选AxisColor，Axis选Z；Style选Points或Squares，Size(m)先设0.02。
- Decay Time设0，程序自己累积，不要再用RViz长时间堆叠旧消息。

## 操作与诊断

先基本水平停稳，确认有一圈点云。再轻微低头5～10度，停稳1～2秒，
接着回水平、抬头5～10度。保持传感器位置基本不变。扫描地面、墙面和箱子。
正确变换后，重叠的墙/地面应大致重合，不是每个倾角都形成一个旋转扇面。
点云只有雷达实际观测到的表面，不会填补被遮挡的箱顶/底面。

在另一个已source ROS的Pi终端：

```bash
ros2 topic echo /tilt_scan/status
```

- `capturing steady scan`：正在采集，points/accepted_scans增加。
- `moving or PNI jump; hold still`：正在转动或姿态跳变，请停稳。
- `waiting for settling history`：等待稳定的连续姿态记录。
- `scan/PNI time mismatch`：配对失败，检查两路数据、时间戳；不要盲目放大门限。
- `waiting for fresh PNI`：检查串口节点、valid和update_tick。
- 旧点还在不表示继续采集，需看accepted_scans是否增加。

清空重试（换位置时必须清空）：

```bash
ros2 service call /tilt_scan/clear std_srvs/srv/Empty '{}'
```

保存PLY（只保存点云，不保存姿态/轨迹，不是ROS地图文件）：

```bash
ros2 service call /tilt_scan/save std_srvs/srv/Trigger '{}'
```

响应返回新文件绝对路径，位于`~/tilt_scan_results/`，时间戳命名，不覆盖旧文件。

## 可选参数

通过脚本传给点云节点，例如：

```bash
bash ~/ros2_ws/start_tilt_scan.sh --ros-args -p sensor_height:=0.5
```

用户当前放宽采集条件、加快显示的启动方式：

```bash
bash ~/ros2_ws/start_tilt_scan.sh --ros-args \
  -p max_rate_deg:=20.0 -p settle_time:=0.4 -p publish_rate_hz:=10.0
```

需要上传新版`pni_tilt_cloud_preview.py`再重启。大量点云增加CPU/网络开销，
若变卡，可将`publish_rate_hz`改成5.0。发布率参数在启动时生效，不支持运行时热改。

sensor_height单位m，仅当已知固定雷达中心离地0.5m时才设置。不能补偿手持升降。
`mount_roll_deg/mount_pitch_deg/mount_yaw_deg`描述从雷达坐标到PNI坐标的安装旋转；
`roll_sign/pitch_sign`默认1，仅在核实PNI约定后可设-1。
`use_heading:=true`使用PNI相对初始航向，仍不是融合，也不解决平移。
`max_rate_deg`默认5，`settle_time`默认0.6，`pair_gap`默认0.15；速度单位度/秒，时间单位秒。

## 验证状态

11项新增主机测试通过：三轴旋转、俯仰高度、移动后等待恢复、重复/失效数据、
时间检查、容量上限、安装旋转、MCU重启、PointCloud2二进制布局、清空与独立PLY保存。
ROS包装用模拟消息验证，并非真实Jazzy运行或物理精度验收。
下一步先用RViz看点云，确认几何和坐标方向，再实施表面高度提取与2.5D栅格。

消息格式依据：[PointCloud2](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/PointCloud2.msg)、
[LaserScan](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg)。
