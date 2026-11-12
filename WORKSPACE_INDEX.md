# 工作区索引（2026-10-09）

本文件只说明文件用途，不替代运行验证。树莓派目前已用 RF2O + SLAM Toolbox 建图；Flask 的 `/map/meta`、`/map/image` 已在局域网验证，用户确认 cpolar 对 5000 端口的穿透正常。

## 当前工作入口

| 用途 | 文件或目录 | 说明 |
| --- | --- | --- |
| STM32 / Keil 工程 | `407_gps初始化失败 - 副本/MDK-ARM/112.uvprojx` | 当前工程；不要与 `v1` 或 `backups` 中的副本混淆。 |
| STM32 传感器 ROS2 节点 | `stm32_pni_ros2_node.py` | 树莓派部署源文件；默认不发布 odom/TF，由 RF2O 负责里程计。 |
| Flask 服务 | `raspberry_pi_temp_serial_service.py` | 当前源文件；含 `/sensor`、`/map/meta`、`/map/image`、`/robot/pose`、摄像头和识别接口。位姿接口于 2026-10-13 本地实现，尚待部署验收。 |
| 小程序小车绘图辅助 | `miniprogram_robot_pose.js` | Canvas 2D 坐标换算、箭头绘制；需复制到实际小程序源码中接入。 |
| 小车位姿接入说明 | `ROBOT_POSE_API.md` | 数据格式、上传启动命令、小程序绘图示例及验收步骤。 |
| PNI 朝向融合试验 | `pni_yaw_fusion_preview.py`、`pni_yaw_fusion_core.py` | 独立朝向滤波与三箭头对照，允许 status=0 为本轮实验假设；不发布 TF，不改变现有地图。 |
| 朝向融合试验说明 | `PNI_YAW_FUSION_TRIAL.md` | 上传、启动、RViz 和 CSV 对照步骤；后续 2.5D 所需数据。 |
| 手持倾斜点云试验 | `pni_tilt_cloud_preview.py`、`pni_tilt_cloud_core.py` | 固定雷达中心假设下用PNI俯仰/横滚转换并累积三维点云；只停稳采集，不发布TF，不使用yaw融合。主机测试通过，实物待验收。 |
| 一键点云试验 | `start_tilt_scan.sh`、`PNI_TILT_CLOUD_TRIAL.md` | 启动串口、雷达、点云三个进程；须先停原建图。RViz查看、清空与PLY保存步骤。尚未生成2.5D高度栅格。 |
| STM32 串口分帧 | `stm32_serial_protocol.py` | Flask 与串口逻辑依赖。 |
| 一键建图 | `start_mapping.sh` | 本地源文件；树莓派上部署名称为 `~/ros2_ws/start_mapping_rf2o.sh`。 |
| Python 测试 | `tests/test_python_bridge.py` | 传感器桥接及地图 PNG 转换的主机测试。 |

## 文档和辅助脚本

- `PROJECT_HANDOFF_20261008.md`：较新的项目交接记录；其中“地图接口尚未实现”的描述已被当前 Flask 文件及实测结果取代。
- `AGENT_HANDOFF.md`：更早的背景与 RF2O/Cartographer 建图记录；以实际代码和新实测为准。
- `二维雷达建图启动操作手册.docx`：分终端启动说明；当前优先使用 `start_mapping.sh`。
- `build_mapping_manual.py`：生成上述 Word 手册；内含工作区绝对输出路径，因此保留在根目录。
- `docx_render_v2/`、`render_manual_pdf.py`：手册预览与渲染辅助材料；不是树莓派运行程序。
- `STM32_FIFO测试说明.md`：STM32 调试历史。
- `项目使用说明书.md`、`项目使用说明书.docx`：早期说明；其中 GPIO UART/`ttyAMA0` 等内容已过时，不应作为当前接线依据。
- `demo_app/`：历史演示程序，未纳入当前建图/Flask 启动链路。

## 历史与生成物（全部保留）

- `v1/`：早期完整 STM32 工程与 Python 版本。
- `backups/`：按调试阶段保存的历史工程、代码和测试；`backups/loose_versions/` 收纳原先散落根目录的 `.bak` 文件。
- `raspberry_pi_temp_serial_service_v1.py`：早期版本，内容与 `v1/raspberry_pi_temp_serial_service.py` 相同；为兼容旧交接文档而保留原位。
- `tests/stm32/build/`、Keil 工程下的编译输出、`__pycache__/`、`docx_render_v2/`：生成物；本次整理不删除。
- `keil_review_build.log`：旧编译日志，保留原位。
- `work/`：现为空目录，保留原位。

## 不在本工作区内

树莓派上的 `~/ros2_ws/slam_params.yaml`、cpolar 配置及微信小程序源码不在这里。需要修改它们时，应先核对树莓派或小程序端的实际文件，不应从本地旧文档推断。
