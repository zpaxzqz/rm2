# ChArUco 板识别与位姿解算

视觉组校内赛作业。识别节点使用 C++、ROS 2 Humble 和 OpenCV 4.5.4。离线回放与现场相机使用同一个识别节点，均可在 Foxglove 查看调试图和板中心位姿。

## 实现

1. 对输入图像检测 `DICT_4X4_50` 标记，在调试图上绘制边框和 ID。位姿估计只使用 `config/board.yaml` 中配置的板内 ID。实物板为 6 × 6 格，方格边长 0.15 m，标记边长 0.12 m，ID 为 0–17。
2. 节点使用最近收到的兼容 `CameraInfo` 内参和畸变系数。至少取得四个 ChArUco 内角点时优先使用内角点。内角点不足时，至少需要两个板内标记，再使用其四角作为观测。无可用标定时仍检测 ID 并输出调试图。
3. 将平面三维点移到以板中心为原点的坐标系。用 IPPE 求平面 PnP 候选，用 LM 对候选细化，并选取重投影误差最小且深度为正的解。拒绝退化点集和 RMS 重投影误差超过 3 px 的解。
4. 有效位姿沿用图像时间戳，旋转发布为单位四元数。解算失败时，识别状态记录最近一次成功位姿及其原始时间戳。调试图显示 `STALE`、位姿年龄和上次有效 XYZ。图像超过 0.5 s 未到达时，状态额外报告 `input_stale=true`。

素材中的实物板从正面看，最上行 ID 0、1、2 分别在从左数第 2、4、6 格，第二行从第 1 格开始交错。通过 `legacy_pattern: 0` 适配这块板。若换成 OpenCV 4.5.4 默认棋盘，在板配置中设为 `1`。

短暂漏检时，若本帧仍解出有效板位姿，曾在最近 120 ms 内识别且完整位于画面的标记会按本帧位姿补画绿色边框，并以 ~ID 标出。tag_ids 始终只包含本帧实际读出的 ID。

## 坐标系

原点位于板中心。以 ID 0–2 在上方、标记正面朝向观察者为准，红色 +X 沿板面向右，绿色 +Y 沿板面向上，蓝色 +Z 垂直板面朝向观察者。板坐标系为右手系。

输出位姿表示板坐标系到 `camera_frame` 光学坐标系的变换。相机 +X 向右、+Y 向下、+Z 向前，调试图的 XYZ 数值是板中心在相机坐标系中的位置，单位为米。

## 编译与启动

在 WSL Ubuntu 22.04 中进入仓库根目录。构建需要 ROS 2 Humble、colcon 和包含 ArUco 模块的 OpenCV 4.5.4。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build
source install/setup.bash
```

使用发放的 AVI 回放时，在当前电脑上运行以下命令。换机器时修改 `video_path`。launch 默认加载配套的 DA3113952 标定。

```bash
ros2 launch board_recognition recognition.launch.py source:=video \
  video_path:="/mnt/d/bighomework/test_media/Video_20261122192405053.avi" \
  loop:=false
```

现场接入发布原始图像及对应内参的海康相机节点后，使用 live 模式。

```bash
ros2 launch board_recognition recognition.launch.py source:=live
```

可传 `show_window:=true` 开启 OpenCV 预览，按 Esc 或 Q 关闭，调试图话题会继续发布。可用 `board_config:=/绝对路径/board.yaml` 替换板配置。回放其他相机素材时，通过 `camera_info_url:=file:///绝对路径/标定.yaml` 指定视频对应的标定。现场模式从相机话题读标定，使用全幅针孔模型，支持 `plumb_bob` 和 `rational_polynomial`。

| 方向 | 话题 | 消息 |
| --- | --- | --- |
| 订阅 | `/camera/front/capture` | `sensor_msgs/msg/Image` |
| 订阅 | `/camera/front/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 发布 | `/recognition/board_pose` | `geometry_msgs/msg/PoseStamped` |
| 发布 | `/recognition/debug_image` | `sensor_msgs/msg/Image`（`bgr8`） |
| 发布 | `/recognition/board_observation` | `interfaces/msg/BoardObservation` |

输入订阅和调试图使用 Sensor Data QoS。每个收到的图像发布一条识别状态，成功帧才发布位姿。Foxglove 可通过 Image 面板查看 `/recognition/debug_image`，并在 3D 面板中选用 `camera_frame` 为固定坐标系、订阅 `/recognition/board_pose`。

## 测试结果

在 WSL Ubuntu 22.04、ROS 2 Humble、OpenCV 4.5.4 下，`camera_frame`、`interfaces` 和 `board_recognition` 三个包均通过构建。

以发放的 DA3113952 标定运行 C++ 识别算法，三张 BMP 的结果如下。

| 图片 | 检出标记数 | 板中心 XYZ（m） |
| --- | ---: | --- |
| `Image_20261122192618491.bmp` | 16 | (-0.143, -0.066, 3.517) |
| `Image_20261122192626869.bmp` | 12 | (0.091, 0.014, 3.472) |
| `Image_20261122192652741.bmp` | 10 | (-0.016, 0.196, 4.611) |

对 `Video_20261122192405053.avi` 逐帧离线处理，1,944 帧均检出标记，1,918 帧通过位姿检查（98.66%）。有效帧的 Z 范围为 3.572–3.725 m。该比例表示解算通过率，位姿绝对误差需要用带真值的测试测量。

ROS 2 视频模式以源文件约 98.95 FPS 回放，测试订阅端在一次运行中收到 642 条逐帧识别状态、630 条有效位姿和 602 张 `bgr8` 调试图，回放结束后收到输入超时状态。实时链路的接收数受 Sensor Data QoS、队列深度和处理速率影响，逐帧算法统计以离线结果为准。

Live 模式通过独立启动的相机话题发布器验证。重复发送三张 BMP 并插入黑帧，确认缺标定时仍能检出 ID，黑帧时不发布新位姿且保留上次位姿的原始时间戳，重新见板后恢复发布，停止输入后报告超时。海康实机和 Foxglove 可视化留待现场验收。
