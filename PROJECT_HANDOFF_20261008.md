# 农业巡检机器人项目交接

更新日期：2026-10-08。下一位接手者先读本文件，再检查当前代码。本文替代旧 AGENT_HANDOFF 中的进度描述；旧文仅作为历史背景。

## 1. 当前进度与本次交接目的

用户最新确认：

- PNI 已能持续接收到数据。
- STM32 稳定性测试已“大致完成”。尚未提供完整量化验收日志，不要把它写成所有工况已经验证。
- 目前通过雷达已经可以建图。不要把项目状态退回“只能看到点云、尚未建图”。

下一阶段适合处理树莓派接收和服务整合，让 STM32 数据可靠进入 ROS2 与 Flask，并与已有建图系统共存。用户本次只要求交接文档，没有要求马上实施这些后续改动。

目前不知道已成功建图所用的软件包、launch 命令、参数文件、里程计来源和 TF 发布关系。接手时优先收集现有可工作的配置，不要推定已经安装或使用 slam_toolbox、rf2o、robot_localization 等特定包，更不要直接替换已工作的建图链路。

## 2. 工作区与操作约束

- 工作区：`C:\Users\33615\Documents\New project111`
- 活跃 STM32 工程：`407_gps初始化失败 - 副本`
- 工程入口：`407_gps初始化失败 - 副本/MDK-ARM/112.uvprojx`
- 用户要求修改前备份，保留可回退版本。
- 禁止批量删除文件或目录；只能一次删除一个明确路径的文件。若确实需要批量清理，停止并交给用户手动操作。
- 不删除 `v1`、`backups`、`.bak_before_*` 等历史文件。
- 保留用户已验证的 DS18B20 推挽 GPIO 模式，不擅自改变接线或波特率。
- 最新实物测试结果来自用户；本地编译与主机模拟测试不能冒充实物验证。

## 3. 硬件与通信

项目由 STM32F407、PNI MM-Amber、DS18B20、树莓派、USB 摄像头和思岚 C1M1 二维雷达组成，小车没有轮速编码器。

| 部分 | 当前已知信息 |
| --- | --- |
| STM32 | MDK 工程选择 STM32F407ZETx；硬件具体丝印仍以实物为准 |
| PNI | I2C1，PB6/SCL、PB7/SDA，代码默认 7 位地址 0x28 |
| 温度 | DS18B20，PA0 |
| USART1 | 115200，printf 启动及调试输出 |
| USART2 | PA2/TX、PA3/RX，115200，主要传感器 JSON 与临时 FIFO 输出 |
| USB-TTL | PA2 接转换器 RX，GND 共地；单向上传无需接另一根 TX/RX |
| 树莓派 | 用户历史环境 Ubuntu 24、ROS2 Jazzy；用户名 rsp，历史 IP 192.168.137.23 |
| USB 设备 | USB-TTL 曾为 /dev/ttyUSB0，雷达曾为 /dev/ttyUSB1，重启可能交换 |
| 雷达 | 思岚 C1M1，USB 接入；用户已确认能建图 |

用户曾反馈树莓派 GPIO 串口引脚烧黑，后来改用 USB-TTL。不要默认恢复树莓派 GPIO UART 接线。最近一次明确测试方式是 USB-TTL 接电脑，尚未确认现在是否已接回树莓派。

## 4. STM32 当前实现

主要业务文件为 `Core/Src/main.c`、`pni.c`、`sensors.c` 和 `Core/Inc/pni.h`。2026-09-08 修复没有更改原 MDK 工程文件、USART 引脚配置或 GPIO 推挽模式。

主循环：

```c
Sensor_UploadTask();
PNI_UpdateTask();
DS18B20_UpdateTask();
PNI_TestUploadTask();
```

2026-10-08 核对源码的参数：

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| PNI_ORIENTATION_DEFAULT_RATE_HZ | 10 Hz | 配置的姿态输出速率 |
| PNI_MOTION_SENSOR_RATE_HZ | 10 Hz | 配置的加速度输出速率 |
| PNI_UPDATE_POLL_MS | 100 ms | STM32 查询/读取间隔 |
| SENSOR_UPLOAD_INTERVAL_MS | 50 ms | JSON 目标发送周期 |
| PNI_DATA_TIMEOUT_MS | 500 ms | 缓存测量超时失效 |
| PNI_FIFO_TEST_ENABLE | 1U | 临时 FIFO 原始输出仍开启 |

这些是源码配置，不是最新实测频率。10 Hz 的新测量通过目标 20 Hz JSON 发送，连续两包重复读数是可能的；不能把串口行数或诊断文本数量当作测量频率。

### FIFO 修复的最终状态

- 芯片内部 FIFO 容量没有被扩大；4096 字节是 STM32 的批次读取缓冲区。
- 单次 I2C 读最多 50 字节，读完本批长度，中途不反复刷新传输计数。
- 采用直接查询 BytesRemaining 的轮询入口。曾添加 IntStatus.bit0 硬门槛，后因兼容性风险撤回，不要直接重新加回。
- 另有 64 字节流解析窗口，逐字节推进，保留未完成包。它与旧版“塞进 64 字节后才解析、截掉前面数据”的缓存逻辑不同。
- 已同步时按 SID 长度解析。时间戳 252/253 各 3 字节，META 254 为 4 字节，姿态 3 与加速度 1 各 8 字节。
- 失去同步后，需要连续至少 3 个完整且检查通过的非填充包，其中至少 2 个是姿态/加速度包，才恢复边界。此为恢复策略，不是协议自带 CRC 保证。
- 不再因一个异常片段拒绝整批正常测量。有效姿态、加速度分别更新；旧数据超过 500 ms 失效。
- I2C 读取失败、超长批次、长时间中断后会清理流残片，避免接上不连续数据。

### 已修正的错误假设

上一版认为一次 FIFO 读取一定从完整包头开始、在包尾结束，遇到错误拒绝整批。用户截图显示读取 132/132 字节，但首字节是 8E，末尾为 `03 4E`，中间仍有可解释的姿态和加速度。原代码在偏移 0 报 UNKNOWN_SID 后，把有效数据也挡住，导致多数 JSON 为 valid:false 和 0。

最终采用跨批次流解析和同步恢复。用户随后确认“一直接收到 PNI 数据”，现又确认稳定性测试大致完成。不要恢复此前“任一包错误就整批清零”的实现。

截图中段还有异常字节。底层 I2C/FIFO 是否存在丢字节、重复字节，尚未有完整实测证明；不能把恢复连续输出等同于彻底证明底层数据无误。

### 串口与温度

- USART2 改为 HAL_UART_Transmit_IT，中断发送，静态发送缓冲区；UART 忙时不覆盖缓冲区。
- PNI/accel 新增 update_tick：STM32 开始读取该批 FIFO 的 HAL 毫秒计数，不是芯片原生时间戳。旧测量重复发送时此值保持不变。
- 姿态单位为度；加速度换算为 `raw * 9.8 / 8192`，单位 m/s²，包含重力，静止某轴约 9.8 是正常情况。
- status 0/1/2/3 分别表示不可信/低/中/高可信度。当前 valid 主要代表格式与时效有效，不等于已经充分校准。
- PNI 无效时目前发送 valid:false，数值用 0 占位。下游必须检查 valid，不能把这些 0 当测量。
- TIM4 当前为 1 MHz，delay_us 已从等待 us*2 修正为等待 us 个计数。
- 温度转换采用分阶段等待，读完整 9 字节并校验 CRC；无有效温度发送 `temperature:null`，不再把默认值当作正常测量。

示意 JSON，数值并非本次实测：

```json
{"tick":10000,"temperature":24.06,"unit":"C","pni":{"valid":true,"heading":110.54,"pitch":0.87,"roll":22.96,"status":3,"update_tick":9960},"accel":{"valid":true,"x":3.957,"y":-0.141,"z":9.325,"status":3,"update_tick":9960}}
```

### 临时测试输出

PNI_TestUploadTask 通过 USART2 约每秒输出 FIFO BEGIN/DATA/END，数据直接复制自实际读取，不额外消耗一次芯片 FIFO。BEGIN 包含 reason、carry、dropped：

- carry：等待补齐或确认的字节数；出现半包不一定是错误。
- dropped：累计因失步/过期丢弃的字节数，重点观察增长速度。
- PARTIAL：等待半包；WAIT_SYNC：等待确认边界；RESYNC：同步恢复。
- INIT_FAILED、EMPTY_FIFO、COUNT_READ、FIFO_READ 等用于诊断初始化及读取问题。

接回树莓派前，应先把 PNI_FIFO_TEST_ENABLE 设为 0U 并重新编译，或者烧录无诊断文本版本。不要让旧 Python JSON 解析器直接接收混合 FIFO 文本。

## 5. 固件、备份与测试

| 路径 | 用途 |
| --- | --- |
| tests/stm32/build/firmware_fifo_test/112.hex | 带临时输出的完整构建固件 |
| tests/stm32/build/firmware_normal/112.hex | 关闭临时输出的同一套修复固件 |
| tests/stm32/run_host_tests.cmd | MSVC + 模拟 HAL，测试真实 pni.c/sensors.c |
| tests/stm32/build_firmware.ps1 | 从 MDK 工程读取源文件/宏/路径，使用已有 ARMCC 独立构建 |
| STM32_FIFO测试说明.md | 详细测试过程、诊断说明及回退方式 |
| backups/review_fix_20260908_214439 | 改动前完整 STM32 工程及两个 Python 文件 |
| backups/pni_receive_diagnosis_20260908_220434 | 轮询入口修正前的代码、测试、说明及 HEX |
| backups/pni_stream_fix_20260908_221133 | 最终流解析修正前的代码、测试、说明及 HEX |
| v1 | 更早期历史整包，非最新稳定版 |

注意：这些备份大多是“修复前”状态，不要把 pni_stream_fix 目录名误当成已修复的最终版。准备下一轮修改时，应另存当前用户已验证版本及对应 HEX。

旧 `MDK-ARM/112/112.hex` 没有被独立构建自动覆盖，不能据名字推定它是新版。

已有验证：两种固件曾完整编译、链接并生成 HEX；主机测试包括混合包、328 字节读取、截图 132 字节按所有 1～132 字节分段回放、半包续接、异常恢复、超时/计数回绕、温度 CRC、发送缓冲区保护及诊断输出还原。保留了原 GPS 文件的末尾换行警告。

先前 Keil IDE 构建遇到设备包/CMSIS 缺失；已有 ARMCC 5 工具链可以独立构建，不要未经检查就重装环境或改 MDK 文件。

工作区 PowerShell 重建命令：

```powershell
& .\tests\stm32\run_host_tests.cmd
& .\tests\stm32\build_firmware.ps1
& .\tests\stm32\build_firmware.ps1 -Production
```

## 6. 手册依据与原文件

用户确实提供过 PNI 模块手册，原位置：

`C:\Users\33615\xwechat_files\wxid_nct4khl4dpcv12_6138\msg\file\2026-05\ENGP-168_MM-amber模块应用手册_V3(1)(1).pdf`

关键页：第 20 页 6.4.2 为轮询/分段读取流程；第 21 页表 7、8 为包结构与长度、单位和 status；第 24 页表 11 为 FIFO 解析示例。第 24 页存在时间戳之后多一个 0x03“准确度”字节的表述不一致，不要据此擅自修改包长度。

配套示例为 `demo_app/Src/mm_loop.c`、`mm_util.c`、`mm_types.h`。读取手册、示例和实机日志时应区分各自能证明的内容。实际日志已出现可解释的 FC 时间戳包，不能再认为“未调用启用时间戳函数，所以一定没有时间戳”。

## 7. 树莓派与 ROS2 待处理问题

目前本地两个 Python 文件尚未应用 9 月审查提出的修复，不要因 STM32 已稳定而认为树莓派端也已完成。

### raspberry_pi_temp_serial_service.py

提供 /sensor、/debug/serial、/snapshot、/video_feed、/recognition/upload、/recognition/image、/recognition/status。历史摄像头及 YOLO 识别图片上传已做过调试，但本地未见完整 YOLO 推理脚本和小程序源码，当前部署行为需核对。

待处理：

- 默认设备仍为 /dev/ttyAMA0；使用 USB-TTL 时需显式指定正确设备。
- readline 超时半包没有累计保留，可能造成 JSON 解析失败。
- 串口打开就标记在线，缺少基于最后有效数据的失联超时。
- 当前将 temperature:null 当作缺少字段报错，与新版 STM32 不兼容；温度无效不应阻止 PNI 数据更新。
- 图片上传缺少鉴权、大小限制及图片内容验证，恢复外网穿透前需处理。

### stm32_pni_ros2_node.py

默认 /dev/ttyUSB0、115200；发布 /pni/heading、/pni/rpy、/pni/imu、/pni/odom、/pni/path、/stm32/sensor_json、/pni/status、/pni/heading_marker，并发布动态 odom -> base_link。

待处理：

- 同样存在串口半包问题。
- 超过 45 度的跳变被拒绝后，比较基准不恢复，可能一直拒绝新的正常角度。
- 定时给缓存姿态打当前时间继续发布，掩盖测量停止；需结合 update_tick 和接收时间判断新鲜度。
- 未正确利用新版 update_tick 区分新测量与重复发送的缓存。
- 加速度位置积分默认关闭。若开启，当前仅用 yaw 旋转 XY，未实现完整重力补偿和实际偏置校准，不能作为可靠导航里程计。
- PNI 状态质量阈值、传感器安装方向和 ROS 坐标关系仍需结合实物核对。

### 两端整合

Flask 与 ROS2 当前分别直接打开串口。不要同时让两者读取同一个 USB-TTL。后续应选定一个串口采集者，通过话题或本地接口分发数据；具体方案需结合现有树莓派部署选定，尚未实现。

已有雷达建图可能也在发布 odom -> base_link。启动旧 PNI 节点前必须检查 TF 所有者，避免两个节点争发同一变换。不要给已有正常 SLAM 强行接入目前的纯加速度积分位置。

## 8. 建议下一步

1. 收集树莓派当前成功建图的启动命令、配置、节点/话题与 TF，明确已有 odom 的来源。保持该链路可回退。
2. 保存当前用户已验证的 STM32 源码及对应固件。关闭临时 FIFO 输出后接回树莓派；先核实 USB 设备身份。
3. 先修复 Python 串口接收、temperature:null 兼容与超时状态，再处理角度过滤恢复、重复测量识别和质量状态。
4. 统一串口读取入口，让 Flask 与 ROS2 共存。不要为了同时启动两个脚本去争抢串口。
5. 在已有建图链路中核对 PNI 坐标系与姿态方向，逐步验证辅助定位；不从头重复搭建已工作的雷达建图。
6. 最后恢复识别图片上传、小程序与外网访问的整体验证。

用户最新说稳定性测试大致完成，不应再要求从最开始机械重复测试。只有出现新问题、改动相关代码或缺少关键验收证据时，再做针对性复测。

## 9. 常用部署信息

以下是历史连接地址，执行前确认树莓派地址仍有效。Python 尚待修改，不要把下面的复制命令当作修复已经完成。

```powershell
scp "C:\Users\33615\Documents\New project111\raspberry_pi_temp_serial_service.py" rsp@192.168.137.23:~/raspberry_pi_temp_serial_service.py
scp "C:\Users\33615\Documents\New project111\stm32_pni_ros2_node.py" rsp@192.168.137.23:~/ros2_ws/stm32_pni_ros2_node.py
```

树莓派侧用于只读核对的命令：

```bash
ls -l /dev/serial/by-id/
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_link
```

建图运行步骤见第 10 节。实际成功建图的 SLAM 命令尚待补录；以下雷达驱动命令不能替代 SLAM 启动命令。

## 10. 建图程序使用与重启恢复

2026-10-08 补充：此前文档缺少操作步骤。用户已确认雷达可以建图，但本工作区没有保存成功建图的 SLAM 启动脚本、配置和里程计启动命令。下述已知步骤用于恢复环境及检查；不能将其描述为完整且已验证的建图启动流程。

### 10.1 登录并加载环境

电脑 PowerShell：

```powershell
ssh rsp@192.168.137.23
```

树莓派每个新终端都先加载 ROS2 和工作区：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

确认 USB 设备身份和雷达可执行程序：

```bash
ls -l /dev/serial/by-id/
ros2 pkg executables rplidar_ros
```

历史上 USB0 是 USB-TTL，USB1 是雷达。以当前实际设备为准，避免打开 STM32 的串口作为雷达。

### 10.2 雷达驱动

旧交接记录中的直接启动建议如下。只有上一条命令确实列出 rplidar_composition，且端口属于雷达时，才适用：

```bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB1 \
  -p serial_baudrate:=460800 \
  -p frame_id:=laser \
  -p angle_compensate:=true \
  -p scan_mode:=Standard
```

历史还使用过 rplidar_c1_launch.py；首次启动曾报可执行程序不存在，重新编译后用户确认能运行，但具体最终参数没有保存在本地。优先保留树莓派上当前成功使用的启动方式。

在另一个已加载环境的终端检查：

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

雷达驱动产生 /scan；看到扫描点并不能单独证明地图已发布。

### 10.3 启动已有建图链路

按树莓派上当前成功使用的方式启动所需里程计、TF 和 SLAM。以下信息必须补录后，下一位接手者才能准确复现：

| 必须记录的项目 | 当前状态 |
| --- | --- |
| SLAM 软件包、launch 或脚本的完整命令 | 尚未记录 |
| 参数文件的树莓派绝对路径 | 尚未记录 |
| 里程计节点及启动命令 | 尚未记录 |
| base_link 到雷达的安装变换及发布者 | 尚未记录 |
| 实际地图话题和地图坐标系 | 尚未核对；/map 与 map 仅为常见名称 |
| 地图保存、加载命令及文件路径 | 尚未记录 |

不要自行添加第二个 odom -> base_link 发布者。已有雷达安装 TF 时也不要重复发布；旧记录中的全零静态 TF 只是调试示例，不能代表实物安装位置。

可在成功运行建图的树莓派上收集信息：

```bash
history | grep -Ei 'ros2 launch|ros2 run|slam|rf2o|cartographer|map_saver|\.sh'
ps -eo pid,args | grep -Ei 'ros2|slam|rf2o|cartographer|launch'
ros2 node list
ros2 topic list -t
```

历史记录可能包含失败或过时命令，要结合当前进程、脚本和用户确认判断。取得实际命令后，将其完整补入本节，不要只记录包名。

### 10.4 RViz 查看地图

树莓派图形桌面终端执行：

```bash
rviz2
```

仅通过普通 SSH 连接时，此命令不保证出现窗口；使用树莓派本地桌面或已有远程桌面。

建图运行后，在 RViz 设置：

1. Fixed Frame 选择实际地图坐标系，通常为 map。
2. 添加 Map，选择实际地图话题，通常为 /map。
3. 添加 LaserScan，选择 /scan。
4. 添加 TF，用于核对小车和雷达的坐标关系。

单独测试雷达时可将 Fixed Frame 设为 laser；看地图时应切回地图坐标系。若实际 frame_id 不同，按消息和 TF 中的名称设置。

若实际话题为 /map、机体坐标系为 base_link，可检查：

```bash
ros2 topic info /map -v
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map base_link
```

Map 显示报错时，检查地图是否发布、QoS 是否匹配，以及地图坐标系到小车/雷达的 TF 是否连续。已能建图的情况下不要用旧 PNI 加速度积分替换现有里程计。

### 10.5 下一阶段：小程序显示地图

用户最新目标是把已有地图显示到小程序。建议 ROS2 桥接节点订阅实际 OccupancyGrid 地图，转换 PNG，并保留宽高、分辨率、原点和版本；Flask 提供图片和元数据，小程序使用 Canvas 显示。

建议新增 /map/image、/map/meta、/robot/pose，当前不能视为这些接口已经存在。小车位置使用 map -> base_link 的有效变换，不能直接把 PNI 姿态当地图位置。先完成局域网地图显示，再处理外网与小程序合法域名。
