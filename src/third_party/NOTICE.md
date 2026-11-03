# 源码来源与改编记录

本工程包含下列 MIT 开源设计/实现的改编，许可原文位于 licenses/。原仓库未修改。

| 当前文件 | 来源 | 改动 |
|---|---|---|
| src/bof_vision/core/src/detection/tongji_detector.cpp、yolov5_decode.cpp | TongjiSuperPower/sp_vision_25，tasks/auto_aim/yolos/yolov5.cpp 与 armor.cpp | 保留模型预处理、logit 解码、角点重排、类别映射；移除 YAML/日志/图像存盘和传统角点纠正依赖；增加模型形状与有限值检查、显式颜色映射；NMS 前过滤敌方颜色与 not_armor，使用浮点框，不按英雄/基地推断板尺寸 |
| src/bof_vision/core/include/bof_vision/latest_buffer.hpp | SZURPVision/RP-26AutoAim-Frame，src/core/utility/threads/inc/latest_buffer/latest_buffer.hpp | 单槽覆盖语义；去除通信框架基类及嵌套发布订阅封装；增加 close、唤醒退出与覆盖计数 |
| src/bof_vision/core/include/bof_vision/orientation_buffer.hpp | Alliance/rmcs_auto_aim_v2，src/io/capturer/imu_snapshot_buffer.hpp | 保留按时间包围与四元数 slerp 思路；改为标准容器，移除 RMCS 消息、组件和环形缓冲依赖；增加时钟域、最大插值间隔、输入验证；不保留主机 fallback 钳制 |
| src/bof_vision/core/include/bof_vision/recorder.hpp、src/bof_vision/core/src/io/recorder.cpp | TongjiSuperPower/sp_vision_25，tools/recorder.hpp、tools/recorder.cpp | 沿用异步 MJPG 视频、逐帧元数据和限频录制；替换为本项目深圳来源的 LatestBuffer，close 后排空退出；移除 fmt/logger 与未接入的四元数，保存来源帧号/时间；补充输出目录、写入异常和剩余空间检查 |
| src/bof_vision/apps/calibrate.cpp | TongjiSuperPower/sp_vision_25，calibration/calibrate_camera.cpp | 沿用对称圆点板生成物点、calibrateCamera 与逐图重投影流程；增加棋盘格、现场原图/视频输入、分辨率/图数检查及可直接加载的 YAML 输出；不默认固定 k3 或沿用原队板尺寸 |
| src/bof_vision/core/src/pose.cpp | TongjiSuperPower/sp_vision_25，tasks/auto_aim/solver.cpp | 沿用平面四点 IPPE 与重投影流程；显式配置对应物理尺寸，适配本项目 TL/TR/BR/BL 顺序及板坐标，保留双解误差信息、正深度和重投影门限；不移植云台/世界变换与 yaw 优化假设 |
| src/bof_vision/core/src/geometry.cpp | RPS auto_aim_rps26，src/rm_auto_aim/armor_detector/src/traditional_detector/detector.cpp，TraditionalDetector::isArmor | 改编灯条中心间距 / 平均长度的大小板比例选择；范围和物理尺寸改为显式 YAML，重叠区保留为歧义，不复制号码分类器；许可见 licenses/RPS-MIT.txt |
| src/bof_vision/apps/live.cpp 相机恢复流程 | TongjiSuperPower/sp_vision_25，io/hikrobot/hikrobot.cpp | 沿用故障后释放设备再打开的恢复流程；合入已有采集线程，增加截止时间、序列号固定、等待界面、关联重置及录像分段；不复制 USB reset 与额外守护线程 |
| src/bof_vision/core/src/tracking/pixel_tracker.cpp | TongjiSuperPower/sp_vision_25，tasks/auto_aim/tracker.cpp 的 state_machine | 改编 detecting/tracking/temp_lost 流程到像素关联；暂失采用秒超时，当前帧未匹配就输出无效；补充类别连续确认与匹配歧义状态，不移植其整车模型、全向切换或发射逻辑 |

`src/bof_vision/apps/live.cpp` 将已有检测管线、海康采集、深圳来源 LatestBuffer 与上述同济录像器改编组合成现场调试入口。界面和启动脚本为本项目适配代码。相机手动曝光/增益设置与同济 `io/hikrobot/hikrobot.cpp` 中关闭自动模式、设置 ExposureTime/Gain 的接入方式一致；不复制其设备 VID/PID、150 FPS 等硬件默认值。

深圳0526/0708后端的类别布局参考 Alliance 的 `src/module/detector/models/shenzhen.hpp`，改编到共享22列解析器；第8类显式映射为基地，而非同济的非装甲板。保留 raw_class，并将第7/8类的小/大基地语义传为 size_hint。相应权重取自用户提供的南理工仓库，未声称深圳框架示例包含这些权重。

RPS 前哨站模型现已按下表改编；仅移植状态结构与几何/预测关系，不代表原仓库完整行为或实机效果复现。

模型权重来源另见 models/README.md。OpenVINO、OpenCV、Eigen、ROS 和 MVS 是外部依赖，其许可证不由本工程 MIT 许可证替代。

像素运动估计直接调用 OpenCV `cv::KalmanFilter`（video 模块），用二维恒速状态辅助关联；矩阵设置、像素噪声初值和接口组合为本项目适配代码，未声称是同济三维 EKF 的移植或参数复现。

集中配置参考同济 `configs/sentry.yaml` 与 `tasks/auto_aim/yolo.cpp` 的 yolo_name 选择入口。本项目将后端、权重、设备和阈值组织为命名组，后台准备、有限缓存和参数原位更新由本项目实现；未复制同济硬件标定、设备参数或宣称已经支持其 YOLOv8/YOLO11 解码。

2026-11-02 空间算法及 ROS 相机恢复接入：

| 当前文件 | 上游来源 | 具体改编与边界 |
|---|---|---|
| src/bof_vision/core/src/aim/tracker.cpp | TongjiSuperPower/sp_vision_25 tasks/auto_aim/target.cpp | 11 维车辆旋转状态、交替半径/高度、恒速转移与白噪声加速度 Q、装甲板几何及其 XYZ/yaw Jacobian；更新使用 Eigen LDLT 与 Joseph 协方差形式，增加配置门限、匹配歧义、超时和数值检查；未移植上游全向目标选择及完整控制 |
| 同文件的 outpost 分支 | auto_aim_rps26 armor_tracker/src/motion/outpost_model.cpp | 10 维前哨站状态、固定半径、三块板独立高度差与预测结构；半径显式配置，角速度由观测估计，不复制固定转速与规则常量；观测改为 XYZ/法向 yaw |
| 同文件 solve_aim | TongjiSuperPower/sp_vision_25 tools/trajectory.cpp | 无阻力低抛物线解；将低角根有理化以减少相消，增加飞行时间迭代、可达性、朝向与时效限制；重力/弹速/延迟配置化，不复用别队弹速 |
| src/bof_vision/src/hik_camera_node.cpp | Tongji io/hikrobot/hikrobot.cpp 与本项目已有 live 恢复代码 | 故障释放/重开、限定重试、固定序列号、重连代次和诊断，未复制 USB reset |

上述 Tongji 与 RPS 文件分别遵循 licenses/TongjiSuperPower-MIT.txt、licenses/RPS-MIT.txt。aim_node 组合现有 Alliance 来源 OrientationBuffer 做姿态插值；ROS 节点、消息、配置及字段验收脚本是本项目适配代码。它们未包含任何实际电控协议。

2026-11-02 火控与串口接入：

| 当前文件 | 上游来源 | 具体改编与边界 |
|---|---|---|
| src/bof_vision/core/src/aim/fire.cpp choose_plate/solve_aim | Tongji tasks/auto_aim/aimer.cpp choose_aim_point、aim | 低速锁板、来板/去板窗口及前哨 70°/30° 默认值；窗口间隙改为预瞄下一块来板并禁止开火；角度和延迟配置化，不复制同济偏置/延迟数值 |
| 同文件 Shooter | Tongji tasks/auto_aim/shooter.cpp；RPS rm_angle_solver angle_solver.cpp isGimbalAngleOk | 命令稳定与云台跟随判据；容差改为按板半宽/半高和距离计算并限幅；增加决策 fire_enable |
| src/bof_vision/core/src/aim/tracker.cpp vote_direction/pin_spin | RPS armor_tracker outpost_model.cpp calDirection | 间隔比较投票、跳变过滤；阈值改为角速度，先判静止票（原实现静止票不可达），窗口滚动并需两次一致才改判 |
| 同文件目标选择 | Tongji tasks/auto_aim/tracker.cpp 优先级切换 | 优先级列表 + 决策优先/锁定/排除；切换需连续帧确认 |
| src/bof_vision/core/{include/bof_vision/gimbal_link.hpp,src/io/gimbal_link.cpp} | Tongji io/gimbal/gimbal.{hpp,cpp}、tools/crc.cpp | SP 帧布局与 CRC16 表；改用 POSIX termios、流式找帧、无帧 2 秒重开；pitch 符号可配置；不依赖 serial 库 |
| src/bof_vision/src/aim_node.cpp 命令线程、core/src/aim/engine.cpp step | Tongji tasks/auto_aim/multithread/commandgener.cpp | 固定频率预测到当前时刻并发送；命令逻辑在不依赖 ROS 的 AimEngine 中；不移植 MPC/规划器 |
| src/bof_vision/apps/handeye.cpp | Tongji calibration/calibrate_handeye.cpp、calibrate_robotworld_handeye.cpp | 云台零平移的手眼模型、圆点板 PnP、cv::calibrateHandEye；改为 rosbag 输入并按图像时刻插值姿态、静止帧与视角去重筛选、棋盘格支持，Tsai 初值后联合重投影最小二乘，输出一致性指标与 aim.extrinsics |
| src/bof_vision/core/src/detection/keypoint_decode.cpp、tongji_detector.cpp 的 v8/v11 分支 | Tongji tasks/auto_aim/yolos/yolov8.cpp、yolo11.cpp、armor.hpp、classifier.cpp | 416/640 预处理、转置输出、关键点排序、38 类表、tiny_resnet 取图区域与 softmax；改为本项目标签和 TL/TR/BR/BL 顺序，敌方颜色在 NMS 前过滤，只把明确的大小板类别作为 size_hint，不移植 ROI 裁剪、存图和传统角点纠正 |
| src/bof_vision/core/src/io/gimbal_link.cpp cboard 协议 | 本队云台固件 sentry_gimbol_new Task/Src/Auto_aim.c、GimbolControl.c | 帧格式、CRC16 表与角度约定按固件实现，不是开源改编；固件为本队代码 |
| scripts/start_match.sh、scripts/system/* | Tongji autostart.sh / watchdog 思路 | 改为 systemd 用户服务 + launch respawn + udev 固定名 |
