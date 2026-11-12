# 农业巡检机器人

STM32F407 采集 PNI MM-Amber 姿态、加速度与 DS18B20 温度，通过 USB-TTL 发送到树莓派。树莓派运行 ROS2 雷达建图和 Flask 服务，向小程序提供传感器、地图、位姿与图像接口。

## 当前组成

| 部分 | 入口 | 用途 |
| --- | --- | --- |
| STM32 固件 | `407_gps初始化失败 - 副本/MDK-ARM/112.uvprojx` | Keil 工程，PNI FIFO 读取与串口 JSON 上传 |
| ROS2 串口节点 | `stm32_pni_ros2_node.py` | 读取 STM32，发布传感器话题 |
| 串口分帧 | `stm32_serial_protocol.py` | 保存半行并提取完整 JSON |
| 雷达建图启动 | `start_mapping.sh` | RPLIDAR、RF2O、SLAM Toolbox 启动链路 |
| Flask 服务 | `raspberry_pi_temp_serial_service.py` | 传感器、地图、位姿、摄像头与识别图片接口 |
| 小程序绘图辅助 | `miniprogram_robot_pose.js` | 地图坐标转换与小车箭头绘制 |
| 姿态融合试验 | `pni_yaw_fusion_core.py`、`pni_yaw_fusion_preview.py` | 独立对照试验，不代替已有定位链路 |
| 倾斜点云试验 | `pni_tilt_cloud_core.py`、`pni_tilt_cloud_preview.py`、`start_tilt_scan.sh` | 停稳采集的倾斜点云试验 |

当前雷达建图链路为 RF2O + SLAM Toolbox。PNI 融合、倾斜点云等试验的实测状态见各自说明，不能将试验功能视作已经完成的可靠惯导或三维 SLAM。

## 硬件和环境

- STM32F407、PNI MM-Amber、DS18B20。
- 树莓派 Ubuntu 24.04、ROS2 Jazzy。
- 思岚 C1M1 USB 二维雷达、USB 摄像头、3.3V 逻辑 USB-TTL。
- PNI：J7-1 SCL 接 PB6，J7-2 SDA 接 PB7，J7-3 接 3.3V，J7-4 接 GND。
- STM32 上传：USART2 PA2 TX 接 USB-TTL RX，并共地；115200、8N1。
- DS18B20 数据脚接 PA0。

USB-TTL 和雷达的 `/dev/ttyUSB*` 编号可能变化，先用 `ls -l /dev/serial/by-id/` 核对。

## 使用入口

1. 打开 Keil 工程编译、烧录，先在电脑串口验证连续 JSON。
2. 将 STM32 接树莓派，安装 ROS2 及 `rplidar_ros`、`rf2o_laser_odometry`、`slam_toolbox` 等依赖。
3. 将 ROS2 Python 节点、`stm32_serial_protocol.py` 和建图脚本部署到树莓派工作区。`start_mapping.sh` 的历史部署名为 `~/ros2_ws/start_mapping_rf2o.sh`。
4. 配置实际串口、雷达安装变换和 SLAM 参数，再按操作手册运行。树莓派的 `~/ros2_ws/slam_params.yaml` 不在本仓库，克隆仓库不等于可以直接启动全部系统。
5. Flask 默认通过 ROS 话题接收传感器数据；不要让多个进程竞争读取同一个串口。具体环境变量和接口见源码及专题说明。

主要接口：`/sensor`、`/map/meta`、`/map/image`、`/robot/pose`、`/snapshot`、`/video_feed` 和 `/recognition/*`。外网访问历史上使用 cpolar 转发 Flask 端口，配置及账号不包含在仓库中。

## 说明文档

- [工作区索引](WORKSPACE_INDEX.md)：程序分工、部署入口和历史文件说明。
- [位姿接口](ROBOT_POSE_API.md)：小程序位姿接入与验证。
- [PNI 朝向融合试验](PNI_YAW_FUSION_TRIAL.md)。
- [PNI 倾斜点云试验](PNI_TILT_CLOUD_TRIAL.md)。
- `PNI十轴模块数据读取与调试说明.docx`：当前整理的 PNI 读取说明。
- `二维雷达建图启动操作手册.docx`：建图启动参考。
- `STM32_FIFO测试说明.md`：FIFO 修复与固件测试历史。

微信小程序完整源码、YOLO 模型及完整推理程序、树莓派运行配置不在本仓库。历史说明可能包含旧配置，使用时以当前代码和实际设备为准。

## 主机测试

```bash
python -m unittest discover -s tests -p "test_*.py"
node tests/test_robot_pose_canvas.js
```

STM32 还有 `tests/stm32/run_host_tests.cmd` 和 `tests/stm32/build_firmware.ps1`；后者使用已有 ARMCC 5 工具链，本机路径见脚本。主机测试不代替树莓派、传感器和小车实物验证。

## 仓库范围

保留当前源代码、工程配置、测试、参考示例和正式说明文档。历史备份、旧版 Word、编译缓存及文档渲染文件通过 `.gitignore` 排除，仍保留在本地。

STM32 驱动、厂商示例及其他第三方代码保留各自版权和许可声明；本仓库没有为这些代码重新授予许可。
