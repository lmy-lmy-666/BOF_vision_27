# 使用说明

项目在 `/home/lmy/BOF_vision_27`。日常流程：编译 → 改 `src/bof_bringup/config/sentry.yaml` → 启动。改 C++ 要重新编译，改 YAML 只需重启。代码结构和算法约定见[开发交接](DEVELOPMENT.md)。

## 1. 编译

Ubuntu 24.04 + ROS 2 Jazzy，补充依赖：

```bash
sudo apt install python3-colcon-common-extensions libopencv-dev libeigen3-dev libyaml-cpp-dev \
  ros-jazzy-cv-bridge ros-jazzy-diagnostic-msgs ros-jazzy-rosidl-default-generators \
  ros-jazzy-ament-cmake-gtest ros-jazzy-launch-ros ros-jazzy-rclpy ros-jazzy-rosbag2-cpp \
  ros-jazzy-rosbag2-storage-mcap ros-jazzy-rqt-image-view python3-yaml
```

| 用途 | 命令 |
|---|---|
| 无设备演示 | `bash scripts/build.sh` |
| 实车（神经网络 + 海康） | `bash scripts/build.sh -DBOF_WITH_OPENVINO=ON -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake -DBOF_WITH_MVS=ON -DMVS_ROOT=/opt/MVS` |
| 加单测 | 在上面命令后加 `-DBUILD_TESTING=ON` |

`build.sh` 每次都会把没写的后端设为 OFF，所以实车版每次都要带全参数。关掉 MVS 或 BOF_BUILD_LIVE 时，之前编出的 `hik_camera_node`、`bof_live` 会从 build/ 和 install/ 里删除，不会残留旧二进制（核心库是静态链接，旧二进制里是旧代码）。没有 `hik_camera_node` 时 `start_match.sh` 直接报错退出（退出码 78，systemd 不会反复重启）。MVS_ROOT 填车载电脑上的 SDK 位置（本开发机是 `/home/lmy/桌面/auto_aim_rps26/src/rm_auto_aim/armor_detector/hikSDK`）。没有 TensorRT 后端，OpenVINO 可用 CPU 或 Intel 核显（device: GPU）。

新终端先 `source scripts/env.sh`。移动项目目录后要重新编译，symlink-install 的产物不能直接拷走。

## 2. 现场窗口（只看检测，不含自瞄）

```bash
bash scripts/run_live.sh --synthetic --yolo light_pair           # 无设备演示
bash scripts/run_live.sh --video /absolute/path/video.avi --yolo tongji_yolov5
bash scripts/run_live.sh --camera auto --yolo shenzhen_0526       # 海康；多相机时填序列号
bash scripts/run_live.sh --camera auto --headless --record        # SSH 无桌面，持续录制
```

| 按键 | 功能 |
|---|---|
| q / Esc | 退出并收尾录像 |
| y | 循环切换 YOLO 组（后台加载，失败保留当前模型） |
| r / s | 开始/停止录像；保存当前原图和叠加图 |
| b | 切换敌方红/蓝 |
| + / - | 调置信度 |
| [ / ] | 调海康曝光 |

先在 MVS 厂商工具里确认相机能正常取图。不要沿用别队的曝光、分辨率和标定。

## 3. 配置文件和检测模型

| 文件 | 用途 |
|---|---|
| `config/sentry.yaml` | 唯一日常调参入口：模型、相机、PnP、自瞄全部参数 |
| `config/match.yaml` | 比赛用，继承 sentry.yaml，只关调试图、改串口固定名 |
| `config/geometry.yaml` | PnP 用的大小板尺寸和各兵种板型 |
| `config/vision.yaml` | 无权重的 ROS 演示配置 |
| `config/geometry.example.yaml` | 尺寸为 0 的空模板 |

加载顺序：默认值 → extends 的基础文件 → 当前文件 → 启动参数。相对路径按声明它的文件所在目录解析。未知或重复的字段会直接报错，不会被悄悄忽略。

检测模型在 `yolo_models` 里按名字分组，`yolo_name` 选当前组，启动时也可用 `yolo:=名字`：

| 组名 | 格式 | 备注 |
|---|---|---|
| tongji_yolov5 | 640 输入，22 列四点 | sp25 默认模型 |
| tongji_yolov8 | 416 输入，只分颜色 | 数字由 tiny_resnet 分类器给出（classifier、classifier_confidence） |
| tongji_yolo11 | 640 输入，38 类 | 颜色 × 兵种 × 大小板 |
| shenzhen_0526 / 0708 | 640 输入，22 列 | 南理工仓库提供 |
| light_pair | 灯条几何 | 无权重，只用于演示 |

权重不进源码包，用 `bash scripts/dev/import_model.sh /path/to/sp_vision_25` 和 `scripts/dev/import_shenzhen_models.sh` 按固定 SHA 导入。各模型的评估情况见[模型说明](../models/README.md)。只支持这几种输出格式，别的 YOLO 权重不能直接替换。

## 4. ROS 启动

```bash
source scripts/env.sh
ros2 launch bof_bringup vision.launch.py                         # 无设备合成演示（vision.yaml）
ros2 launch bof_bringup vision.launch.py \
  config:="$PWD/src/bof_bringup/config/sentry.yaml" source:=hik aim:=true
```

| 参数 | 取值 |
|---|---|
| source | synthetic / video（配 video_path）/ hik / external（用已有的 image_topic，如 bag 回放） |
| aim | true 时启动 aim_node（需 aim.enabled=true） |
| respawn | true 时节点崩溃 1 秒后重启（比赛用） |
| yolo、enemy_color、camera_serial、exposure_us、gain | 覆盖配置文件中的对应项 |

| 话题 | 内容 |
|---|---|
| camera/image_raw | 相机图像，stamp 为主机收帧时间 |
| perception/candidates、pixel_track、armor_poses | 检测框、像素关联、PnP 位姿（相机 optical 系） |
| debug/image | 检测调试图（有订阅时才画） |
| tracking/spatial_target、aim/solution | 跟踪状态、解算与开火原因 |
| aim/debug_image | 自瞄调试图：检测框 + 预测装甲板 + 瞄准点 |
| gimbal/imu、gimbal/state、gimbal/command | 串口反馈和发出的命令，供录包和曲线 |
| diagnostics | 各节点状态、串口帧数、CRC 错误、命令线程超时次数 |

## 5. 相机内参标定和测距

固定镜头、对焦、分辨率后，在现场窗口按 s 采集多角度标定板原图（覆盖中心和边缘、多个距离，至少 10 张）：

```bash
ros2 run bof_vision bof_calibrate --images /absolute/path/live_session \
  --pattern chessboard --cols 9 --rows 6 --spacing-mm 25 \
  --output data/calibration/camera.yaml --review-dir data/calibration/review
```

棋盘格的 cols/rows 是内角点数，圆点板是圆心数，间距必须实测。只支持 OpenCV 针孔模型。拟合 RMS 小不等于标定准确，换镜头、对焦、分辨率后要重新标定。

标定后在 sentry.yaml 填 `calibration_path` 和 `geometry_path: geometry.yaml`，PnP 才会开启。geometry.yaml 里是 sp25 的尺寸（小板 135 mm、大板 230 mm、灯条 56 mm），量过实物后再修正。

单距离测距检查：隔离一个目标，量出相机光心到板中心的斜距，录一段现场会话，然后运行：

```bash
python3 scripts/field_report.py --session /absolute/path/live_session --label 6 \
  --reference-distance-m 3.0 --max-relative-error 0.05 --min-samples 30 --output field_report.json
```

## 6. 自瞄配置

aim 段按功能分组，每组前都有注释：

| 组 | 内容 |
|---|---|
| 顶层 | enabled、origin、timestamp_source、camera_latency_s、各频率、debug_image_rate_hz |
| extrinsics | 相机→云台四元数、相机和枪口在云台系的位置（手眼标定得到） |
| gimbal_link | 串口来源、协议（cboard / sp）、端口、是否使用串口弹速和颜色 |
| targets / selection | 各兵种模型和半径；无决策时的目标优先级 |
| aimer | 弹速、控制延迟、枪管偏置、选板窗口 |
| outpost | 前哨转速、方向投票、暂失保留时间 |
| shooter | 是否自动开火、开火容差 |
| decision | 是否接入决策（默认 false） |
| tracking | EKF 噪声、关联门限 |

启用条件：aim.enabled=true，填好 extrinsics 和 aimer.muzzle_speed_mps。timestamp_source 实车用 host_compensated，并填实测的 camera_latency_s；如果是 host_receive，节点能跟踪，但不会给出瞄准结果。

默认不接导航和决策（decision.enabled=false）：开火只看 shooter.auto_fire 和开火判据，热量由电控控制。联调决策时才打开，接口见[开发交接](DEVELOPMENT.md)第 6 节。

## 7. 上车标定和测延迟

按顺序做，前一步的结果是后一步的输入。录包统一用 `bash scripts/record.sh 预设 [名字]`，文件存到 artifacts/bags/。

**① 手眼标定（相机→云台）**

aim_node 要有外参才能启动。标定前先填理想安装值，只为接通串口、发布姿态：

```yaml
extrinsics: {camera_to_gimbal_q_xyzw: [0.5, -0.5, 0.5, -0.5], camera_in_gimbal_m: [0, 0, 0], muzzle_in_gimbal_m: [0, 0, 0]}
```

aimer.muzzle_speed_mps 先填估计值；同时设 shooter.auto_fire=false，并设 decision.enabled=true、decision.require_directive=true 且不发决策，这样视觉不会控制云台。

固定标定板，用遥控把云台停在 10 个以上不同的 yaw/pitch 组合，每个停 1 秒以上（两个轴都要变化）：

```bash
bash scripts/record.sh handeye
ros2 run bof_vision bof_handeye --bag artifacts/bags/<时间>_handeye --calibration data/calibration/camera.yaml \
  --pattern circles --cols 7 --rows 5 --spacing-mm 30 --output handeye.yaml
```

把输出的两个外参填进 aim.extrinsics。合格标准：reprojection_rms_px 在 1 px 以内，board_centre_max_spread_m 为毫米级。偏大说明姿态和图像不同步，或视角太少。muzzle_in_gimbal_m 用尺子量。

**② 相机延迟 camera_latency_s**

对着静止装甲板左右、上下摆动云台：

```bash
bash scripts/record.sh latency
ros2 run bof_vision bof_aim_replay --bag artifacts/bags/<时间>_latency --config src/bof_bringup/config/sentry.yaml \
  --mode latency --min 0 --max 0.06 --step 0.001 --label 3
```

把输出的 best 值填进 camera_latency_s，timestamp_source 改为 host_compensated。原理：延迟对了，静止目标在参考系里的方位才不会跟着云台晃。

**③ 云台跟随和发弹延迟**

让视觉控制云台跟随目标，录 `bash scripts/record.sh aim follow`，然后：

```bash
ros2 run bof_vision bof_aim_replay --bag artifacts/bags/<时间>_follow --config src/bof_bringup/config/sentry.yaml --mode command_lag
```

aimer.control_delay_s 从"云台滞后 + 发弹延迟"起调，最终以场地上打旋转目标的命中情况微调。当前固件的发弹计数固定为 1，发弹延迟暂时测不出。

**④ 回放**

训练或比赛时录 `bash scripts/record.sh aim`，改代码或参数后离线复现：

```bash
ros2 run bof_vision bof_aim_replay --bag artifacts/bags/<时间>_aim --config src/bof_bringup/config/sentry.yaml --output replay_out
```

输出 tracks.csv（每次跟踪更新和先验残差）、commands.csv（100 Hz 的解算、开火原因、实际云台指向），终端汇总前哨转向判定时刻和每块板的 z 残差。三块板 z 残差均值接近 0，说明高度差和板编号正确。回放与 aim_node 用的是同一个 AimEngine。

无设备时可以用 `python3 src/bof_vision/test/integration/aim_tools_check.py OUTPUT_DIR src/bof_bringup/config/sentry.yaml` 生成答案已知的合成 bag 检查这些工具。

## 8. 上车第一次、比赛启动和录包

**上车第一次（只开云台，不开火）**：sentry.yaml 里设 aim.enabled=true、shooter.auto_fire=false，填外参（未标定时先用理想值）、calibration_path、geometry_path。

```bash
ros2 launch bof_bringup vision.launch.py config:="$PWD/src/bof_bringup/config/sentry.yaml" source:=hik aim:=true
ros2 run rqt_image_view rqt_image_view /aim/debug_image
```

调试图：绿框是本帧匹配的板，黄框是可打的其他板，灰框是前哨还没看到过的板，紫圈是旋转中心，红十字是瞄准点（开火时加圈）。上方两行文字是跟踪状态、转速、转向，以及解算状态和开火原因。

依次确认：
1. 云台对着静止目标时，预测板与检测框重合。不重合先查外参和 camera_latency_s。
2. 目标在画面右侧时云台向右转，在上方时向上抬。反了说明 C 板安装方向或 pitch 正负与固件假设不同，停下来找电控。
3. 前哨旋转时，状态行的 spin 与实际转向一致。

三项都对了再开 auto_fire。

**比赛启动**：

```bash
bash scripts/system/install_autostart.sh   # 车上运行一次：udev 固定名 /dev/bof_gimbal、systemd 用户服务、开机不登录也启动
systemctl --user start bof-vision          # 手动启动；journalctl --user -u bof-vision -f 看日志
systemctl --user disable --now bof-vision  # 关闭自启
bash scripts/start_match.sh                # 不装服务时直接运行；BOF_CPUS=2-5 绑核，给雷达/导航留核
```

start_match.sh 等串口出现后，用 match.yaml 以 respawn 方式启动相机、检测和自瞄：单个节点崩溃 1 秒后重启，整个 launch 退出由 systemd 重启。日志在 artifacts/logs/时间戳/。改参数仍然改 sentry.yaml。

**录包**：`bash scripts/record.sh aim|images|handeye|latency [名字]`。aim 不含图像，适合整场录；images 带原图（1440×1080 约 100 MB/s），只录片段。bag 也可以直接用 PlotJuggler 打开看曲线（需另装 ros-jazzy-plotjuggler-ros）。

## 9. 排查

| 现象 | 看哪里 |
|---|---|
| 不出瞄准 | `/aim/solution` 的 status：imu_stale（串口没姿态）、exposure_time_unsynchronized（timestamp_source 是 host_receive）、no_plate_in_window、unreachable |
| 不开火 | fire_status：auto_fire_off、fire_not_enabled、aim_not_enabled、waiting_for_plate（等下一块来板）、command_changed、gimbal_not_on_target |
| 串口不通 | `/diagnostics` 的 gimbal_link、gimbal_frames、gimbal_crc_errors；确认 /dev/bof_gimbal 存在 |
| 跟踪频繁丢失 | tracking/spatial_target 的 status：association_ambiguous、invalid_target_geometry、imu_sync_timeout |
| 预测板不贴检测框 | 外参、camera_latency_s、geometry.yaml 尺寸 |
| 前哨打错高度 | 回放看三块板 z 残差；outpost.min_plate_hits |
| CPU 占用高、掉帧 | 绑核 BOF_CPUS；模型 device 改 GPU；比赛关闭调试图 |
