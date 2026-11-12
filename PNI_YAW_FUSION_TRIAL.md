# PNI 朝向融合对照试验（2026-10-13）

## 2026-10-14：历史偏差锁死修正

旧算法只比较 PNI 与历史融合预测。当 PNI 逐步偏离时，可能连续被接受；
PNI 回归后反而与错误预测相差超过 30°，被持续拒绝。失效、拒绝和时间不匹配时，
旧预测继续保留历史修正，造成橙色箭头远离当前两路输入。

现在保留增量预测，但增加保守保护，不是无约束的标准卡尔曼滤波：

- 使用 **当前 RF2O 与零点对齐的 PNI** 最短角差作门限，默认 30°。
- 正常修正或复用同一 PNI 时，结果约束在当前两路角度之间的最短圆弧内。
  例如 179° 与 -179° 的中间在 ±180° 附近，不在 0° 附近。
- 两路分歧超门限、PNI 失效/超时或时间不配对时，输出当前 RF2O，
  清除历史融合偏差并重置方差；不悄悄重新对齐 PNI 零点来掩盖分歧。
- JSON 新增 `rf2o_pni_delta_deg`、`fallback_to_rf2o`、`arc_clamped`；
  `gain` 是名义卡尔曼增益，约束发生后不代表最终输出的有效权重。
  CSV 列保持兼容，`reason` 记录拒绝/回退和圆弧保护。

两路恢复一致后，新的 PNI 测量可以重新参与修正。回退/约束时橙色箭头可能跳回蓝色，
这是明确的异常保护，不表示真实车体突然转动。保护保证输出范围，不证明精度提高。
蓝色 RF2O 本身仍可能失准，本轮优先信任它是保护策略，不是外部真值判定。

备份：`backups/yaw_guard_fix_20261014/`。16 项主机测试通过；
包括日志中的偏离模式、慢变拉偏、恢复、失效回退、重复测量和 ±180° 环绕。
树莓派实测待部署。本轮需上传 **core 和 preview 两个文件**，使用新的 CSV 文件名。

2026-10-14 修正：初始尚未对齐或 PNI 超时时，隐藏箭头的四元数字段原先使用整数 0/1，
触发真实 ROS Python 消息转 C 的 `PyFloat_Check` 断言。现统一为浮点值，
并增加所有箭头浮点字段的回归检查。更新树莓派上的 `pni_yaw_fusion_preview.py` 后再试。

本轮按用户明确给出的两个假设试验：PNI 与 RF2O 朝向正方向一致（sign=+1），
暂时把 status=0 的 PNI 朝向当作可靠输入。保留原始质量状态，试验结果不作为已校准的证明。

## 实现范围

`pni_yaw_fusion_preview.py` 订阅现有 `/stm32/sensor_json` 和 `/odom_rf2o`，
不打开串口、不发布 TF。原 STM32 串口节点、一键建图、Flask 和 cpolar 继续运行。
因此没有第二个 `odom -> base_link` 发布者，也不会把实验结果悄悄用于地图。

采用 `pni_yaw_fusion_core.py` 中的一维朝向卡尔曼试验算法：

1. 从 RF2O 里程计读取朝向，在首次时间接近且有效的两路输入到达时，
   设置 `offset = RF2O_yaw - PNI_yaw`。初始化期间应让整个车体保持静止。
2. 以 RF2O 两次朝向之差作为预测增量，按假设的漂移噪声增加不确定度。
3. 每个新的 PNI `update_tick` 只用于修正一次；当前 RF2O/PNI 分歧超过 30° 时
   不修正，退回当前 RF2O。正常结果限制在两路输入之间的最短圆弧内。
4. PNI 掉线或不配对时退回当前 RF2O，不保留旧融合偏差；失效的 PNI 对照箭头隐藏。
   RF2O 掉线时不输出新的朝向，
   RViz 箭头在 0.5 秒内消失。MCU 重启或 RF2O 时间中断后重新对齐零点。

这只是朝向试验，不是完整的位置/速度 EKF；不会修正 RF2O 已算出的 x/y。
箭头位置全部借用同一条 RF2O 里程计，仅方向不同。现有 `/robot/pose` 与小程序箭头仍来自
原建图 TF。位置、地图能否因融合改善，需下一阶段接入一致的定位链路后验证。

PNI 样本时间用 ROS 接收时间减去 MCU 的 `tick - update_tick` 估计；
没有消除 UART/网络延迟，不是真正硬件同步。默认只配对相差不超过 0.25 秒的输入。
这里初始静止对齐用于本次相对运动试验，不代表已经完成磁北、地图、安装姿态的完整标定。

## 上传与启动

Windows PowerShell 上传两个新增文件（不覆盖旧建图脚本）：

```powershell
scp "C:\Users\33615\Documents\New project111\pni_yaw_fusion_core.py" "C:\Users\33615\Documents\New project111\pni_yaw_fusion_preview.py" rsp@rspi.local:/home/rsp/ros2_ws/
```

在新的树莓派 SSH 终端中启动，沿用当前建图的 ROS 环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/pni_yaw_fusion_preview.py --ros-args \
  -p heading_sign:=1 \
  -p pni_min_status:=0 \
  -p csv_path:=/home/rsp/pni_yaw_guarded_20261014.csv
```

启动时保持车体静止约 2 秒。终端 `reason` 出现
`initial compass/odom zero aligned` 或 `PNI correction applied` 说明已配对并对齐。
`pni_used=true` 表示该次新 PNI 测量实际参与了修正。
CSV 采用追加方式，已有文件保留；重启试验会产生新的对齐过程，分析时分段比较。
需要独立记录时通过 `csv_path` 指定另一个文件名。

不需要安装额外滤波包。只使用现有 Jazzy ROS 消息类型与 Python 标准库。
如找不到输入话题，先检查当前终端与建图进程是否使用同一 ROS 环境和通信域。

## 在 RViz 观察

在已显示地图的 RViz 中点 Add，添加 **MarkerArray**，Topic 选
`/pni/yaw_comparison_markers`。Fixed Frame 可沿用 `map`，也可以选 `odom`。

| 箭头 | 来源 | 长度 |
| --- | --- | --- |
| 蓝色 | 原 RF2O | 最短 |
| 绿色 | 初始零点对齐后的 PNI | 中等 |
| 橙色 | 融合朝向 | 最长 |

箭头共享同一 x/y，稍微分层并设置不同长度，以便朝向一致时也能区分。
如果绿色不出现，检查 PNI 有效性及时间配对；橙色在初始对齐后出现。
终端每秒打印数值，可另外查看：

```bash
ros2 topic echo /pni/yaw_fusion_preview
```

在 X11 帧率较低时，CSV 与上述话题更适合判断角度变化和延迟，不能只依据显示流畅程度。

## 实测步骤与解释

1. 车体静止 20 秒，记录蓝/绿/橙角度的波动与漂移。保持电机等实际干扰源的状态可记录。
2. 让整个车体缓慢左转 20～30°、停稳，再右转回起始方向；PNI 与雷达必须一起转动。
3. 比较转向响应、是否抖动、回到起点的偏差；记录操作时段。
4. 有空间后再测试直行、转弯和闭合路径。

橙色更平滑不等于更准确。若没有外部已知朝向，只能评价一致性、抖动与重复性，
不能据此认定绝对精度提高。status=0 被放行是本轮实验假设；若结果不佳，重新评估校准和磁干扰。

## 参数

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `heading_sign` | 1 | 用户确认 PNI/RF2O 正方向一致 |
| `pni_min_status` | 0 | 本轮允许未校准状态；可改 2 或 3 |
| `pni_std_deg` | 8.0 | PNI 朝向标准差的初始假设；调小会增加 PNI 权重 |
| `rf2o_drift_std_deg` | 2.0 | RF2O 朝向漂移过程噪声，单位度/√秒；调大会增加 PNI 修正权重 |
| `input_timeout` | 0.5 | 输入数据有效时限，秒 |
| `pair_gap` | 0.25 | 两路估计时间的最大配对差，秒 |
| `max_innovation_deg` | 30.0 | 当前对齐 PNI 与 RF2O 的最大允许分歧；必须小于 180° |
| `odom_topic` | `/odom_rf2o` | 原始雷达里程计 |
| `odom_frame` / `base_frame` | `odom` / `base_link` | 必须与输入消息一致 |

以上噪声参数尚未由真实记录估计，属于试验起点。每次只改变一个参数，重启节点重新对齐，
记录对应试验文件。

停止试验：在该 Python 终端按 Ctrl+C。原建图不受影响。

## 下一阶段：2.5D

本次只验证 yaw，不处理 pitch/roll，也不测量高度。
2.5D 栅格需要每格的高度或高度范围；水平 2D 雷达与 yaw 融合本身不会产生这些数据。
后续需确认雷达是否有已知安装倾角、是否能改变扫描平面，以及雷达外参、完整姿态和高度基准。
只有获得对应的三维点，才能讨论高度栅格，而不能把二维占用 PNG 直接称为 2.5D 地图。

主机数学和 ROS 消息包装测试已完成，树莓派运行与真实融合效果待用户部署、记录。
RViz 消息语义依据：[ROS Marker 定义](https://github.com/ros2/common_interfaces/blob/jazzy/visualization_msgs/msg/Marker.msg)。
