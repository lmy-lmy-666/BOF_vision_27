# 开发交接

更新：2026-11-02（电控保护修改后交接）。工作区 `/home/lmy/BOF_vision_27`。操作步骤见[使用说明](USAGE.md)。

## 1. 现状和约定

**当前阶段：主要软件模块已实现，准备进入实机集成验证。用户已确认电控代码已修改，正在等待电控烧录；尚无完整目标编译结果、烧录版本或板上验证记录。源码修改、主机验证、烧录、实机验收必须分别记录，不能合并写成“已完成”。**

约定：优先复用同济 sp_vision_25、RPS auto_aim_rps26 等开源模块，先做出能上车调试的程序，再按实车问题迭代。参数集中在 sentry.yaml。仿真验证不能记作实车验收，外参、延迟、弹速不用别队数值。哨兵是单相机 + 双雷达，自瞄与导航、决策互不接入。

| 部分 | 已完成 | 未完成 |
|---|---|---|
| 检测 | 5 种模型后端、像素关联、现场窗口、录像、相机重连 | 海康实拍与拔插恢复验收；2027 本队数据评估或训练 |
| 解算 | 内参标定、IPPE PnP、大小板选择、geometry.yaml | 实际镜头标定、多距离测距验收、板尺寸实测 |
| 自瞄 | 车辆/前哨 EKF、前哨方向投票、目标优先级、选板、弹道、开火判据、500 Hz 命令、调试叠加图 | 实测外参/延迟/偏置，场地命中率 |
| 云台 | cboard/SP 接入；电控接收校验、通信过期锁定及停止输出源码已修改，40 项主机隔离检查通过 | 等待电控烧录；目标编译结果待提供；真实串口、停止响应和安装约定未验收 |
| 工具 | 手眼标定、测延迟、bag 回放、自启、录包 | 实车数据上使用 |

固件实际目录：`/home/lmy/桌面/sentry_gimbol_new./sentry_gimbol_new/`，协议及本次修改见第 5 节。用户计划接入实际机器；截至本次交接，尚未收到车载硬件详情、本队标定与录制数据、板上版本及真实反馈验收记录。

## 2. 代码结构

```text
bof_interfaces/msg/          # ROS 消息契约（检测、位姿、跟踪、解算、云台、决策）
bof_vision/
├── core/include/bof_vision/ # 公共接口，核心库不依赖 ROS
├── core/src/
│   ├── detection/           # 检测器工厂、YOLOv5/深圳四点解码、YOLOv8/YOLO11 关键点解码、OpenVINO 适配
│   ├── io/                  # 海康 MVS、相机工厂、录像、云台串口（gimbal_link.cpp）
│   ├── pipeline/            # 检测管线与模型切换
│   ├── tracking/            # 像素关联
│   ├── aim/                 # config（配置）、tracker（EKF）、fire（选板/弹道/开火）、engine（同步与命令）、overlay（调试图）
│   ├── pose.cpp             # IPPE PnP
│   └── geometry.cpp         # 大小板选择
├── src/                     # ROS 节点：hik_camera、source、pipeline、aim
├── apps/                    # live、replay、calibrate、handeye、aim_replay
└── test/                    # unit（gtest）与 integration（仿真端到端、工具检查）
bof_bringup/
├── launch/vision.launch.py
└── config/                  # sentry（调参）、match（比赛）、geometry、vision（演示）、geometry.example
scripts/                     # build、env、run_live、start_match、record、field_report；system/、dev/
```

节点：`hik_camera_node` → camera/image_raw → `pipeline_node`（检测 + PnP）→ perception/armor_poses → `aim_node`（跟踪、火控、串口）。图像经 DDS 从相机节点传到检测节点；跟踪、火控、串口在同一进程。`aim_node` 只做消息转换和线程，同步、跟踪、开火逻辑都在核心库 `bof::AimEngine` 里，可以单测，也能被 `bof_aim_replay` 直接调用。

生成物：build/install/log；运行证据、日志、bag 在 artifacts/。来源声明在 third_party/NOTICE.md，不能删。

## 3. 检测与位姿契约

候选保留 raw_class、size_hint 和当前帧角点（TL/TR/BR/BL）。size_hint 只来自模型明确给出的大小板类别，不按兵种猜测。PixelTrack 是二维关联，不是机器人身份。

| 后端 | 输出 | 标签来源 |
|---|---|---|
| tongji_yolov5 | 1×N×22，640 输入 | 9 类，第 8 类为非装甲，丢弃 |
| shenzhen_0526/0708 | 1×N×22，640 输入 | 第 7/8 类为小/大基地 |
| tongji_yolov8 | 1×14×N，416 输入 | 网络只分红蓝，数字由 tiny_resnet 分类（9 类含 not_armor） |
| tongji_yolo11 | 1×50×N，640 输入 | 38 类表（颜色 × 兵种 × 大小板），只把基地和 3/4/5 的大小板作为 size_hint |

敌方颜色在 NMS 前过滤。live 窗口的模型切换在后台预热，失败保留旧模型；ROS 节点只能在启动时选模型。

ArmorPoses 是当前帧在相机 optical 系（x 右、y 下、z 前）的观测，旋转向量表示板局部系（x 右、y 下、z 法向）到 optical 的旋转。时间戳是主机收帧时间，不是曝光时刻。

## 4. 自瞄算法与坐标约定

数据流：ArmorPoses + 云台四元数 → 按图像时刻插值姿态、外参变换到参考系 → EKF（执行器线程）→ 命令线程按 command_rate_hz 预测到当前时刻 → 选板 + 弹道 → 开火判据 → 串口。配置写作 `组.字段`。

- **参考系**：四元数的地面系，重力对齐、z 向上；原点为云台中心。origin=gimbal_centered（sp25 做法）时原点随底盘移动，底盘平移由目标速度吸收，不接里程计。
- **车辆**：改编 sp25 Target，11 维 `[x,vx,y,vy,z,vz,yaw,ω,r,dr,dz]`，2/4 板交替半径和高度。
- **前哨**：改编 RPS OutpostModel，10 维 `[x,vx,y,vy,z,vz,yaw,ω,dz1,dz2]`，半径固定，三块板高度差在线估计；板编号相对第一块被观测到的板。
  - 方向投票改编 RPS calDirection：每 outpost.vote_samples 个匹配观测，按间隔 vote_interval 的朝向变化率投票。判出方向后把 ω 钉在 ±outpost.speed_radps（规则 0.8π），判静止就钉 0；之后连续两个窗口结论一致才改判（前哨 3 分钟后停转）。
  - RPS 原实现的"静止"票永远投不出来，这里先判静止票。
  - 暂失后保留 outpost.timeout_s，期间按转速继续预测。
- **目标选择**：按 selection.priority 排序；更优目标连续出现 switch_frames 帧才切换。仍是单目标 EKF。
- **选板**：改编 sp25 Aimer。|ω| ≤ aimer.spin_threshold_radps 时在可射角内锁板防抖；高速目标和前哨用来板/去板窗口（前哨 70°/30°）。
  - 两窗口之间预瞄下一块来板，但不开火（in_window=false），这一点与 sp25 不发命令不同。
  - 前哨的板被观测到 outpost.min_plate_hits 次后才能选（高度差已估出）。
- **弹道**：无阻力低抛解 + 飞行时间迭代。总延迟 = 观测年龄 + aimer.control_delay_s（高速目标可单独设）+ 飞行时间。yaw/pitch 是参考系里的方位角和仰角，含枪管偏置。
- **开火**：改编 sp25 Shooter。条件：同一目标同一块板；命令跳变小于 2 倍容差；云台指向（IMU +x 轴）与命令的误差在半块板对应的角度内（RPS isGimbalAngleOk 的思路）；shooter.auto_fire 打开；决策允许（若接入）。

时间戳：host_receive 只跟踪、不出瞄准；host_compensated 用"收帧时间 − camera_latency_s"作曝光时刻；exposure_synchronized 留给真实硬件同步；simulation 仅供合成检查。串口姿态以主机接收时刻（ROS 时钟）入缓冲，按图像时刻做 slerp 插值。

调试叠加图（overlay.cpp）：aim_node 订阅 pipeline 的 debug/image，按同一时间戳和该帧的曝光姿态，把预测装甲板、旋转中心、瞄准点投影回图像，发布 aim/debug_image。debug_image_rate_hz=0 时完全不订阅图像。

## 5. 云台串口协议

默认 `gimbal_link.protocol: cboard`，依据本队固件 sentry_gimbol_new 的 Task/Src/Auto_aim.c、GimbolControl.c、Math/Src/QuaternionEKF.c。USB 虚拟串口：VID 0x0483、PID 0x5740，波特率无效。

| 方向 | 字节数 | 内容 |
|---|---|---|
| 云台 → 视觉 | 43 | 0x53、mode、enemy_color、q[4] wxyz、yaw、yaw_vel、pitch、pitch_vel、bullet_speed、bullet_count、CRC16 |
| 视觉 → 云台 | 28 | 0x50、mode（0 不控、1 控云台、2 控云台并开火）、yaw、yaw_vel、yaw_acc、pitch、pitch_vel、pitch_acc、CRC16 |

角度约定（按固件推得，需上车核对）：
- q 是 BMI088 机体系到地面系的旋转（z 向上）。
- 固件的 INS.Yaw 是机体 x 轴的航向。
- 固件里叫 "Roll" 的量用公式 asin(-2(q1q3−q0q2)) 计算，实际是绕机体 y 轴的标准俯仰，机头向下为正。
- 固件控制把 INS.Yaw 跟到 yaw×57.3°、INS.Roll 跟到 −pitch×57.3°。所以视觉发的 yaw 是枪管方位、pitch 是仰角（向上为正）；反馈的 pitch 要取负才是仰角。
- 这要求 C 板 x 轴沿枪管方向，与本项目"枪管轴 = IMU +x"的假设一致。

### 5.1 电控源码已修改，等待烧录

已核对并修改上述固件目录内的六个文件：`Task/Inc/Auto_aim.h`、`Task/Src/Auto_aim.c`、`Task/Src/ModeControl.c`、`Task/Src/Shoot.c`、`Task/Src/GimbolControl.c`、`Src/usbd_cdc_if.c`。本次未修改云台 PID、既有 VOFA 实现或上报字段的固定值。

| 原问题 | 修改后的源码行为 | 验证状态 |
|---|---|---|
| CRC 结果未阻止数据进入控制；任务重复消费旧接收缓冲区 | 由真实 USB 接收事件处理数据，完整性、CRC、模式范围和有限数值检查通过后才更新快照 | 主机验证通过；实际 USB/板上行为待验收 |
| 目标丢失计时被误当作通信超时 | 根据最后有效接收时间判定过期；消费者读取时也检查；失效锁定视觉控制 | 主机超时、计时回绕及恢复锁定检查通过 |
| AUTO 缺少 Prepare1 处理，撤销开火后可能沿用旧电机输出 | 非发射/未知 AUTO 状态进入停止处理，发送前复查停止条件 | 原问题已隔离复现；修改后主机检查通过 |
| 恢复连接可能继续旧状态 | 启动、USB 初始化/反初始化、无效消息或过期触发锁定；收到数据本身不能解除 | 主机检查通过；板上操作与停止响应待验收 |

新增行为需要明确交接：

- `VISION_SAFETY_TIMEOUT_MS` 当前默认 **50 ms，仅为待实测验证的暂定值**，不能用标称发送周期证明不会误触发。
- 解除锁定要求：固件观察到 `s[0]`、`s[1]` 都在下挡，同时收到新鲜有效的 `mode 0` 中立消息，且没有待完成的部分消息。物理拨杆对应关系仍需核对。
- AUTO 通信失效调用既有云台停止处理，不进入目标丢失后的扫描；有效通信中“没有目标”与通信失效是不同状态。
- RC 下撤销视觉控制后仍可能按遥控逻辑动作；RC 发射拨杆上挡的独立手动路径没有改成依赖视觉通信。此保护不是整机断能，也不能替代独立停止装置和遥控失联保护。
- 发射停止使用既有零速度目标 PID，实际制动电流不必为零。主机替身不能证明物理制动效果。

修改前备份：`/home/lmy/BOF_vision_27_archive/firmware-safety-before-72x98a93/`，包含本次开始时已有改动及备份哈希；未重置或提交固件仓库。视觉工作区没有 Git 元数据，后续不能假定能用 Git 回退整个视觉项目。

本机交接资料（打包视觉源码不会自动包含外部固件目录）：

- [固件修改记录及验证边界](/home/lmy/桌面/sentry_gimbol_new./sentry_gimbol_new/docs/视觉通信与停止保护_修改记录.txt)
- [40 项主机检查结果与源码哈希](/home/lmy/桌面/sentry_gimbol_new./sentry_gimbol_new/docs/vision_safety_host_results.txt)
- [固件主机安全回归脚本](/home/lmy/桌面/sentry_gimbol_new./sentry_gimbol_new/tests/test_vision_safety.py)
- [修改前问题核对与需求说明](电控端安全停止与接收校验修改说明.txt)：历史问题说明，不代表当前源码仍未修改。

### 5.2 仍未验证或未修改的部分

- 用户已确认**等待电控烧录**。本次环境没有 Keil/ARM 编译器；完整目标编译是否通过待电控提供结果，不能写成编译失败或已经通过。
- 尚无实际烧录版本、板上串口、最终总线输出、停止响应及机械行为的验证证据。
- `bullet_speed`、`bullet_count` 的固定值未在本次修改，不能将其视为真实测量；`use_bullet_speed=false` 的既有约定保留。
- `enemy_color` 的既有来源与 `use_enemy_color=false` 约定保留；反馈 `yaw_vel`、`pitch_vel` 仍不能直接视为角速度。
- 正常退出发送中立消息、进程重启、上位机重连，均不能替代下位机停止链路的实机验证。

[旧固件修改建议](FIRMWARE_CHANGES.md)中的简短代码示例早于本次修复，不是当前补丁，不能覆盖现有实现。[原实车测试方案](TEST_PLAN.md)也尚未整体按新保护行为重新验收；其中保护未验收前允许装弹的旧安排已撤回。停止链路完成安全验证前，暂停装弹测试。

`protocol: sp` 保留 sp25 的 SP 协议（两字节 'SP' 帧头，pitch 方向可配）。两种协议都是：2 秒没有有效帧就重开串口；没有目标时发 mode 0；节点退出时发一帧 mode 0。串口模式下 aim_node 发布 gimbal/imu、gimbal/state、gimbal/command 供录包，这些话题不参与控制。

## 6. 决策接口（默认断开）

decision.enabled=false 时，aim_node 不订阅也不发布下面两个话题。视觉不读雷达和里程计，导航也拿不到视觉目标。代价是无敌、锁敌、前哨停转这些信息拿不到。联调时打开：

- 订阅 `/sentry/aim_directive`（AimDirective）：aim_enable、fire_enable、lock_label、priority_label、excluded_labels、outpost_stopped。超过 decision.timeout_s 没收到视为无决策；require_directive=true 时，无决策就不控云台、不开火。
- 发布 `/auto_aim/target`（AutoAimTarget）：跟踪状态、类别、中心位置和速度、yaw/v_yaw、方位和距离、前哨转向、瞄准/开火状态及原因、弹速。frame_id 为 reference_frame，与导航 odom/map 的对齐方式未定。
- 标签编号：0 哨兵、1 英雄、2 工程、3/4/5 步兵、6 前哨、7 基地。rm_interfaces 的 SentryLockEnemy 用 7 哨兵、6 前哨、8 基地，由决策侧转换。

## 7. 现场工具

| 工具 | 输入 | 作用 |
|---|---|---|
| bof_calibrate | 标定板图片或视频 | 相机内参 |
| bof_handeye | 标定板图像 + /gimbal/imu 的 bag | 相机→云台外参。只用 ±0.1 s 内云台静止的帧；Tsai 初值，再联合优化外参和板位姿的重投影误差。只用 Tsai 时，固定板配小幅摆动会差几度 |
| bof_aim_replay replay | 位姿 + 姿态 + 命令的 bag | 用 AimEngine 按 aim_node 相同节拍复现，输出 CSV 和每块板的 z 先验残差 |
| bof_aim_replay latency | 静止目标、摆动云台的 bag | 扫描 camera_latency_s，使目标方位的散布最小 |
| bof_aim_replay command_lag | 命令 + 姿态 + 云台状态的 bag | 云台滞后命令的时间、开火到发弹计数加一的延迟 |
| field_report.py | 现场会话 | 单距离测距误差报告 |
| scripts/record.sh | — | 四种录包预设 |
| scripts/start_match.sh、scripts/system/ | — | 比赛启动、systemd 自启、udev 固定串口名 |

## 8. 验证记录

全部是仿真、离线或主机替身数据，不代表实车效果。以下旧验证记录不等于本次重新执行了全量回归。

本次接手期间新增/核实的证据：

- 结构验收实际运行了 ROS 合成图像链路、红蓝两种 CLI/ROS 配置一致性、12 帧无窗口现场入口，均通过；记录在 `artifacts/output/structure_acceptance_46n2246c/`。
- 发现关闭 MVS 后残留旧海康节点，后续源码已添加关闭 MVS/现场工具时清理相应旧产物的逻辑。重新核对时 MVS 已开启且节点为新产物；本轮没有再执行开关切换测试。
- 电控停止保护修改后，`tests/test_vision_safety.py` 的 40 项主机检查通过。接收模块编译实际源码，模式/输出检查提取实际函数体，PID/CAN 等使用替身；USB 回调接线做静态检查。没有完成完整固件目标编译、烧录或硬件测试。

此前项目记录：

- 单测 34 项（colcon test）：前哨投票和转速钉定、来板窗口、开火判据、优先级/无敌/锁敌、SP 与 cboard 帧 CRC 和方向、AimEngine 的决策过期/弹速/IMU 过期、调试图投影位置、YOLOv8/YOLO11 解码。
- `test/integration/aim_e2e.py`：虚拟串口按 cboard 帧模拟云台（按命令转动），合成旋转前哨。结果：中心误差 <1 cm，判顺时针并钉 −2.513 rad/s，发出开火帧，前哨设为无敌后释放目标，调试图正常，节点正常退出。
- `test/integration/aim_tools_check.py`：答案已知的合成 bag。手眼旋转误差 0.006°、平移 0.06 mm；相机延迟 12 ms、云台滞后 30 ms、发弹延迟 50 ms 都准确恢复；三块板 z 残差均值 <0.5 mm。合成数据没有噪声。
- 配置一致性（CLI/ROS）、ROS 图像链路 smoke、launch respawn（杀掉 pipeline_node 后用新 PID 重启）均通过。
- sp25 demo.avi 上：YOLOv5/YOLO11/YOLOv8 都能跑；PnP 用 geometry.yaml 能给出有效位姿。没有人工标注真值，不代表精度，见[模型说明](../models/README.md)。

证据在 artifacts/output/fire_control_20261102、aim_tools_20261102、yolo_v8_v11_20261102。更早的验证在 artifacts/output/validation。

## 9. 开源来源

| 来源 | 用到的部分 |
|---|---|
| 同济 sp_vision_25 | 检测预处理与 YOLOv5/v8/v11 解码、tiny_resnet、录像、内参与手眼标定、PnP、车辆 EKF 模型、选板、开火判据、命令线程、SP 协议与 CRC16、自启思路 |
| RPS auto_aim_rps26 | 前哨模型与方向投票、大小板比例、板尺寸开火容差 |
| Alliance rmcs_auto_aim_v2 | 姿态插值、深圳模型输出格式 |
| 深圳 RP-26 | 最新值缓冲 |
| 本队云台固件 | cboard 协议与角度约定 |

具体改编和 MIT 许可原文见[来源声明](../third_party/NOTICE.md)。本项目没有整体复现这些上游系统，不能把它们的命中率当作本项目指标。

## 10. 下一步与所需资料

当前阻塞项是等待电控完成编译确认并烧录，以及取得实际设备记录，不是继续大规模调整目录。

1. 电控提供目标编译结果、实际烧录版本标识和对应源码/构建配置；用户当前确认的状态仍为“等待烧录”。
2. 先在执行器断能或隔离环境核对真实接收、失效锁定、状态转换及最终总线输出；再由电控确认设备的安全停止行为和独立停止手段。记录实际响应，不把主机替身结果替代板上证据。
3. 核对车载依赖、相机取图、原始图像质量、标定文件、时间戳和通信间隔。停止链路验收前，暂停装弹测试。
4. 根据可复现的实际问题继续迭代，保留每次运行的有效配置、软硬件版本、日志和原始数据。首次结构验收不代表完整自瞄系统已经验收。

最小资料包：**实际配置文件、完整启动日志、一小段原始图像记录、当前最明显的一个现象及发生时间**。不要求一次收齐全部资料。

| 资料 | 内容与用途 |
|---|---|
| 实际运行配置 | YAML、启动命令、模型名称、有效配置快照，确认真正加载的参数 |
| 软硬件版本 | 车载电脑、系统/ROS、相机镜头与分辨率、视觉及板上固件版本 |
| 原始图像与对应调试图 | 注明光照、曝光、增益和时间，检查模糊、过曝、畸变和输入质量 |
| 相机标定资料 | 标定板类型/实测尺寸、原图、内参与畸变文件，核查标定质量 |
| 同时段记录 | 原始图像、姿态、诊断、错误计数、日志及时间戳，用于通信和同步排查 |
| 性能记录 | CPU/内存、处理耗时、温度、同时运行的其他任务，定位持续负载问题 |
| 具体复现描述 | 预期、实际、出现时间和复现步骤，避免依据经验参数盲调 |

## 11. 开发命令

```bash
# 编译时加 -DBUILD_TESTING=ON
bash scripts/dev/test.sh                 # gtest
bash scripts/dev/smoke.sh                # ROS 图像链路
bash scripts/dev/test_config_ros.sh      # CLI/ROS 配置一致性
python3 src/bof_vision/test/integration/aim_e2e.py OUT_DIR src/bof_bringup/config/sentry.yaml
python3 src/bof_vision/test/integration/aim_tools_check.py OUT_DIR src/bof_bringup/config/sentry.yaml

bash scripts/dev/import_model.sh /path/to/sp_vision_25            # 同济 v5/v8/v11 权重
bash scripts/dev/import_shenzhen_models.sh /path/to/rmcs_auto_aim_v2
python3 scripts/dev/compare_models.py --help                      # 多模型对照（无真值时只看候选数和耗时）
python3 scripts/dev/evaluate_detections.py --help
bash scripts/dev/package.sh                                       # 源码包，不含权重、SDK、生成物、录制
```

修改前的备份在 `~/BOF_vision_27_archive/`（src-before-fire-control、src-before-aim-engine 两个 tar 包，以及更早的 20261102-143657 目录）。

电控主机安全检查（不会连接设备）：

```bash
python3 /home/lmy/桌面/sentry_gimbol_new./sentry_gimbol_new/tests/test_vision_safety.py
```

这是主机隔离检查，不是 Keil 工程构建或烧录命令。
