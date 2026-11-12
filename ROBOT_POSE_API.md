# 小车位姿与小程序地图接入（2026-10-13）

当前 Flask 源文件新增 `GET /robot/pose`。工作区没有实际微信小程序源码，
因此本次提供服务端实现和可复制的小程序绘图模块，尚未部署到树莓派或接入真实页面。

## 数据来源与接口

在 `STM32_DATA_SOURCE=ros` 模式下，与地图、传感器订阅共用 ROS 节点和线程。
通过 TF Buffer 查询 `lookup_transform(map, base_link, Time())`，获得
小车基座在地图坐标系中的位置，自动组合 `map -> odom -> base_link` 链。
如地图消息使用其他 frame，则以收到的 `/map` 的 frame 为准。
使用 PNI 的 heading 不能替代这里的地图定位；本接口也不宣称提供定位精度或协方差。

默认 10 Hz 查询 TF，默认有效时限 1 秒。无 TF、TF 超时、非法数据或 ROS 线程失败：
HTTP 503、`success=false`、`data.valid=false`，坐标和 `pixel` 返回 null，供页面隐藏旧标记。
正常 HTTP 200，响应禁止缓存。TF 的源时间戳与接收时效都检查；反复查到同一个过期 TF
不会让它重新变成有效。用 `tf_age_ms` 查看当前数据年龄。

响应结构示意（数值仅供说明）：

```json
{
  "success": true,
  "data": {
    "valid": true,
    "frame_id": "map",
    "child_frame_id": "base_link",
    "timestamp": "2026-10-13T12:00:00.000",
    "tf_stamp": {"sec": 100, "nanosec": 0},
    "tf_age_ms": 50.0,
    "x": 1.0, "y": 1.0, "z": 0.0,
    "yaw": 1.5707963267948966, "yaw_deg": 90.0,
    "pixel": {
      "x": 20.0, "y": 50.0, "yaw": -1.5707963267948966,
      "in_bounds": true, "width": 100, "height": 80, "map_version": 7
    },
    "error": ""
  }
}
```

- 世界坐标 `x/y/z` 单位为米；`yaw` 为弧度，地图中从 +X 朝 +Y 转动为正。
- `pixel` 以地图 PNG 左上外边缘为原点；x 向右、y 向下，连续坐标可直接用于 Canvas。
  标记的正前方为图形局部 +X，Canvas 使用 `pixel.yaw` 旋转。
- 坐标已处理地图原点的平移、旋转和 PNG 的上下翻转。
- `valid=true` 只表示有时效内 TF，不表示 SLAM 位置绝对准确。
- 未收到地图或 frame 不一致时，世界位姿仍可有效，但 `pixel=null`。
  超出地图范围则 `in_bounds=false`，不要把标记强制夹到边缘。
- `pixel.map_version` 对应接口查询当时已转换的地图。它不是 TF 与地图严格时间同步的保证。

## 部署到树莓派

在树莓派 Flask 所在终端按 Ctrl+C 停止旧服务，保留建图程序运行。
然后在树莓派 SSH 终端备份当前服务（如这个备份名已有文件则不会覆盖）：

```bash
cp -n ~/raspberry_pi_temp_serial_service_map.py ~/raspberry_pi_temp_serial_service_map.before_pose_20261013.py
```

Windows PowerShell 上传当前源文件，覆盖已备份的部署文件：

```powershell
scp "C:\Users\33615\Documents\New project111\raspberry_pi_temp_serial_service.py" rsp@rspi.local:/home/rsp/raspberry_pi_temp_serial_service_map.py
```

原部署的 `~/stm32_serial_protocol.py` 仍需保留。本次没有新增自定义 Python 依赖文件。
新用到 ROS Jazzy 的 Python TF 绑定 `tf2_ros_py`（Python 导入名为 `tf2_ros`）。
它与静态 TF 命令所在的包不同，是否安装需以启动时的导入结果为准。

树莓派启动 Flask（与现有建图同为 Domain 25）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=25 STM32_DATA_SOURCE=ros
python3 ~/raspberry_pi_temp_serial_service_map.py
```

另一个树莓派终端验证：

```bash
curl -i http://127.0.0.1:5000/robot/pose
```

浏览器亦可访问 `http://rspi.local:5000/robot/pose`。
现有 cpolar 仍转发 5000，无需增加隧道；小程序地址为现有 HTTPS 基址加 `/robot/pose`。
若持续 503，先确认 `/map/meta` 正常，然后在同域 ROS 环境检查：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

若出现 Python 模块导入错误，查看 Flask 终端报错；需要时安装发行版匹配的
`ros-jazzy-tf2-ros-py`，不要换用其他 ROS 发行版的包。

可选配置：`ROBOT_BASE_FRAME=base_link`、未收地图时的 `ROBOT_MAP_FRAME=map`、
`ROBOT_POSE_RATE=10`、`ROBOT_POSE_TIMEOUT=1.0`。本项目使用真实 ROS 时间，建图 `use_sim_time=false`。

## 小程序绘图接入

复制 `miniprogram_robot_pose.js` 到小程序的 `utils/` 中。它适用于 `type="2d"` Canvas。
每次重绘先绘制地图，再调用 `drawRobot`；这样不会留下上一次的箭头。

```javascript
const { drawRobot } = require('../../utils/miniprogram_robot_pose.js');
// displayedMeta 为当前真正画出来的 PNG 所对应的 /map/meta 响应的 data。
// ctx 为该地图的 Canvas 2D context，box 为地图显示区域，使用同一坐标单位。
wx.request({
  url: baseUrl + '/robot/pose',
  timeout: 1000,
  success(res) {
    redrawMap(); // 使用页面现有绘图函数，先清空并重画地图
    if (res.statusCode === 200 && res.data.success) {
      drawRobot(ctx, res.data.data, displayedMeta, box);
    }
  },
  fail() {
    redrawMap(); // 网络失败时隐藏旧的小车标记
  }
});
```

这段是页面集成示例，`baseUrl/ctx/displayedMeta/box/redrawMap` 需接入实际页面已有变量。
建议位姿请求每次完成后约 200ms 再发下一次，避免并发积压或旧响应覆盖新位置；
页面隐藏/退出时停止请求。地图仍可约每秒更新，标记可更快更新。
地图绘图应按 `scale=min(box.width/map.width,box.height/map.height)` 等比缩放并居中，
与辅助模块保持一致。如果页面用其他缩放/平移方式，则调用 `projectPose` 后应用页面自己的变换。

地图在建图时尺寸和原点可能改变。辅助模块使用世界位姿和**已显示图片**的元数据重新计算，
避免直接使用服务端新地图的 pixel 在旧图上画错位置。更新图片成功后才替换 `displayedMeta`；
不得将新元数据与旧 PNG 混用。`/map/image?v=...` 当前只有防缓存作用，不保存历史地图：
下载跨过地图更新时刻可能拿到新版本，应核对下载前后 `/map/meta` 的 version，变化时重试。

## 验收

1. `/map/meta` 和 `/map/image` 正常，`/robot/pose` 返回 HTTP 200。
2. 缓慢平移/转动车体，对照 RViz 的基座 TF 检查世界位置和朝向。
3. 小程序箭头位置、方向与 RViz 一致，地图原点变化后也能正确重投影。
4. 停止建图后，位姿接口应转为 503，小程序不再显示旧箭头。
5. 单独中断小程序的网络请求时，页面隐藏旧标记。

主机测试使用 ROS/Flask 桩验证数学、时效、接口状态和 TF 查询参数；
不替代树莓派上实际 ROS 和小程序端的验收。

实现依据：[ROS Jazzy TF 监听教程](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.rst)、
[Python TF 包清单](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros_py/package.xml)。
