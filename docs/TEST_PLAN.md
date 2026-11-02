# 实车测试方案

> **状态更新（2026-11-02）：电控保护源码已修改，等待电控烧录，尚未完成板上停止验证。本方案正文尚未整体按新增锁定行为复核，不能作为装弹测试许可。先完成编译确认和执行器断能/隔离环境验证；当前保护行为与验证边界见[开发交接第 5 节](DEVELOPMENT.md#5-云台串口协议)。**

适用：BOF_vision_27 自瞄首次上车，云台固件 sentry_gimbol_new，单相机（海康）。2026-11-02 编写。

原则：从不动到动、从不开火到开火、从静止目标到旋转目标。每个阶段通过后再进入下一阶段，不通过就停在本阶段排查。每阶段改动的参数都要记录，并保存当时的配置副本。命令细节见[使用说明](USAGE.md)，协议和约定见[开发交接](DEVELOPMENT.md)第 5 节。

## 0. 安全规则（每次开机前读一遍）

依据固件 ModeControl.c / GimbolControl.c / Shoot.c：

| 遥控器 | 云台 | 发射 |
|---|---|---|
| s[0] 下 | STOP，两个云台电机输出 0 | — |
| s[0] 中 | RC 手动；**但视觉发 mode 1/2 时云台跟视觉走** | s[1] 下：停；中：视觉发 mode 2 才打；上：一直打 |
| s[0] 上 | AUTO：跟视觉；视觉丢失 1 秒后回扫描 | 同上 |

（s[0]/s[1] 对应左右哪个拨杆，先用遥控器实测确认，写进第 10 节记录表。）

1. **急停 = s[0] 拨到下**。测试时遥控器始终在手，任何异常先拨下，再处理软件。
2. 阶段 1～6 **不装弹**，s[1] 保持在下；shooter.auto_fire=false。
3. 第一次让视觉控制云台时，云台前方 1 m 内不站人；手不放在云台活动范围内。
4. 固件把视觉 pitch 限在仰角约 −20°～+22°，yaw 不限，云台线缆要能承受连续转动，否则先确认机械限位。
5. 开火测试戴护目镜，弹道方向无人，并确认热量由电控控制。
6. 停止视觉：Ctrl-C 或 `systemctl --user stop bof-vision`。正常退出时节点会发一帧"不控制"，固件随后回到遥控/扫描。
7. **停止保护未完成板上验收前，暂停装弹测试。** 电控源码已增加接收校验、通信失效锁定及停止输出检查，当前等待烧录。原“修复前将发射拨杆保持中挡仍可装弹测试”的安排撤回；接收超时或上位机发送 mode 0 均不能单独证明最终停止已生效。

## 1. 准备（台下，不上电机）

| # | 步骤 | 通过标准 |
|---|---|---|
| 1.1 | 车载电脑安装依赖（使用说明第 1 节），带 OpenVINO + MVS 编译，加 `-DBUILD_TESTING=ON` | 编译无错误；`ros2 pkg executables bof_vision` 列出 hik_camera_node、aim_node、pipeline_node |
| 1.2 | `bash scripts/dev/test.sh`，运行 aim_e2e.py | 34 项单测通过；e2e 打印 spin=clockwise、fire 次数 > 0 |
| 1.3 | 拷入模型：`bash scripts/dev/import_model.sh <sp_vision_25>`；深圳模型同理 | models/ 下有 tongji_yolov5/v8/v11、shenzhen |
| 1.4 | 备份配置：`cp src/bof_bringup/config/sentry.yaml artifacts/field_tests/sentry_初始.yaml` | — |
| 1.5 | `bash scripts/system/install_autostart.sh`，然后 `systemctl --user disable bof-vision`（调试期间不自启） | /etc/udev/rules.d/99-bof-vision.rules 存在 |

## 2. 设备连通（云台断电或 s[0] 下）

**2.1 相机**
- 先用 MVS 厂商工具取图，确认分辨率、帧率、曝光。
- 再运行 `bash scripts/run_live.sh --camera auto --yolo tongji_yolov5`。
- 通过：画面连续；拔掉相机再插上能自动恢复（窗口显示等待后继续）。

**2.2 串口**
- 插上 C 板 USB，`ls -l /dev/bof_gimbal`。没有的话用 `udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'` 核对，固件应为 0483/5740。
- 启动 aim_node（aim.enabled=true，外参先填理想值，见使用说明第 7 节①），s[0] 保持下：
  ```bash
  ros2 launch bof_bringup vision.launch.py config:="$PWD/src/bof_bringup/config/sentry.yaml" source:=hik aim:=true
  ros2 topic hz /gimbal/imu          # 应接近固件发送频率（Auto_aim_Task 约 1 kHz 循环）
  ros2 topic echo /diagnostics       # bof/aim：gimbal_link=connected，gimbal_crc_errors 不增长
  ```
- 通过：频率稳定；CRC 错误为 0 或极少；拔插 USB 后 3 秒内恢复。

**2.3 姿态方向**（云台断电，用手转）
- `ros2 topic echo /gimbal/state`，然后：
  - 枪口向左转 → yaw 增大（地面系 z 向上，逆时针为正）；
  - 枪口抬起 → pitch 增大（已换算为仰角）。
- 用 PlotJuggler 或 echo 看 `/gimbal/imu` 四元数，静止时不漂。
- 通过：两个方向都对。**方向不对就停止，联系电控核对 C 板安装方向**：固件和本项目都假设 C 板 x 轴沿枪管。

**2.4 颜色**：`/gimbal/state` 不含颜色。enemy_color 由固件按 robot_id 给出，没接裁判系统时恒为"敌方蓝"。检测颜色先在 sentry.yaml 的 enemy_color 手动设置。

## 3. 检测与资源（云台不动）

| # | 步骤 | 通过标准 |
|---|---|---|
| 3.1 | 现场窗口按 [ ] 调曝光，灯条不过曝、数字可辨 | 记录曝光值 |
| 3.2 | 对 1、3、7 号和前哨装甲板，在 2/4/6 m 各录一段：`run_live.sh --camera auto --record`，用 y 键切换 5 个模型 | 记录各模型在各距离是否稳定检出、类别是否正确 |
| 3.3 | 选定模型写入 yolo_name | — |
| 3.4 | 雷达和导航同时运行，再起视觉，`top` 看 CPU；`ros2 topic hz /perception/armor_poses` | 检测帧率 ≥ 相机帧率的 90%；导航不掉帧。不达标就试 BOF_CPUS 绑核、模型 device: GPU |
| 3.5 | 看 `/diagnostics` 的 bof/perception：last_processing_ms | 记录 p50（单帧处理耗时） |

## 4. 相机内参和测距

| # | 步骤 | 通过标准 |
|---|---|---|
| 4.1 | 固定对焦和分辨率，拍 ≥15 张标定板（中心、四角、近、远），运行 bof_calibrate | fit RMS < 0.5 px；review 图里角点贴合 |
| 4.2 | sentry.yaml 填 calibration_path、geometry_path: geometry.yaml | pipeline 启动日志显示 PnP=on |
| 4.3 | 量实物装甲板：灯条中心距、灯条长度（大小板各一块），与 geometry.yaml 不同就改 | 记录实测值 |
| 4.4 | 用卷尺量 2、4、6 m 正对的小板，各录 30 帧以上，运行 field_report.py | 每个距离的相对误差 P95 < 5% |
| 4.5 | 4 m 处把板转 30°、60° 再测 | 距离误差 < 5%，状态不是 ambiguous |

距离误差偏大时：先查 geometry.yaml 的尺寸，再查内参；所有距离都偏同一个比例，通常是尺寸填错了。

## 5. 手眼标定（不开火，视觉不控制云台）

配置：decision.enabled=true、decision.require_directive=true、不发决策；shooter.auto_fire=false。

| # | 步骤 | 通过标准 |
|---|---|---|
| 5.1 | 标定板固定在 1～2 m，s[0] 中，用遥控把云台停在 ≥12 个姿态（yaw 至少 3 档 × pitch 至少 3 档，每档停 1 秒以上） | — |
| 5.2 | `bash scripts/record.sh handeye`，然后运行 bof_handeye | views ≥ 10；reprojection_rms_px < 1；board_centre_max_spread_m < 0.005；deviation_from_ideal_mount_deg 与目测安装偏差相符 |
| 5.3 | 外参写入 aim.extrinsics；用尺子量枪口相对 C 板的位置，写入 muzzle_in_gimbal_m | — |
| 5.4 | 重启后用遥控转云台，看 `/aim/debug_image`：静止装甲板上的预测框应贴着检测框 | 遥控慢转时偏差 < 板宽的一半 |

## 6. 相机延迟和方向核对（第一次让视觉控制云台，不装弹）

**6.1 相机延迟**
- 配置不变，对着静止装甲板（3～4 m），用遥控左右、上下摆云台 20 秒，`bash scripts/record.sh latency`。
- 运行 `bof_aim_replay --mode latency`，结果填进 camera_latency_s，timestamp_source 改为 host_compensated。
- 通过：曲线有明显最小值；在最优延迟下 bearing sd < 0.2°。
- 复查：遥控快转时，调试图上的预测框仍贴着检测框。

**6.2 方向核对**：decision.enabled=false（视觉直接控云台），s[1] 下，s[0] 中。
1. 装甲板放在画面中心偏右 → 云台应向右转并停在板上。
2. 放在偏上 → 云台应抬起。
3. 移走目标 → 视觉发 mode 0，遥控重新接管。
4. Ctrl-C 停止视觉 → 云台不应乱甩。
5. s[0] 上（AUTO）重复 1～3；丢失目标 1 秒后应开始扫描。

通过：五项都符合。**任何一个方向反了，立即 s[0] 下**，记录现象后找电控，不要改视觉代码去凑方向。

**6.3 跟随质量**：手持装甲板慢速左右走动，录 `bash scripts/record.sh aim follow`。
- 通过：云台平稳跟随，没有持续振荡。
- 运行 `bof_aim_replay --mode command_lag`，记录云台滞后时间。

## 7. 静止目标开火（装弹）

配置：shooter.auto_fire=true，s[1] 中（视觉发 mode 2 才打）；aimer.muzzle_speed_mps 填当前档位的实测弹速（测速仪或裁判系统）。

| # | 步骤 | 通过标准 |
|---|---|---|
| 7.1 | 3 m 正对静止小板，每次 10 发 | 先看落点偏差，调 aimer.yaw_offset_deg / pitch_offset_deg，直到落点集中在板中心 |
| 7.2 | 在 2、5、8 m 各 20 发，不改偏置 | 命中率逐距离记录；远距离普遍偏低说明弹速或下坠（无空气阻力）有误差 |
| 7.3 | 看 fire_status 统计（回放 commands.csv） | 正对静止目标时 fire 占多数；gimbal_not_on_target 占比高说明开火容差偏紧或云台没稳住 |

只有一个距离偏、其他距离准，先查 muzzle_in_gimbal_m 和弹速。所有距离都偏同一个角度，就调偏置。

## 8. 移动和旋转的车辆

| # | 步骤 | 通过标准 |
|---|---|---|
| 8.1 | 目标车平移（1 m/s 左右），4 m | 调试图里预测框领先实际板；命中率记录 |
| 8.2 | 目标车小陀螺低速（< 2 rad/s） | 锁板不来回切换（调试图里绿框稳定）；命中率记录 |
| 8.3 | 小陀螺高速（4～8 rad/s） | 只打来板窗口内的板，窗口间隙显示 waiting_for_plate；命中率记录 |
| 8.4 | 调 aimer.control_delay_s：从 6.3 测得的滞后起，每次 ±5 ms，看 8.3 的命中率变化 | 记录最优值；高速和低速差别大时设置 high_speed_* |

每一项都录 `record.sh aim`，赛后可以回放复现。

## 9. 前哨站

需要场地前哨站，或者能按 0.8π rad/s 转动的三板转台（三块板高度不同、半径 0.2765 m，按 2027 图纸核对）。

| # | 步骤 | 通过标准 |
|---|---|---|
| 9.1 | 静止前哨：outpost.spin=auto，观察 30 秒 | 状态行 spin=stopped，ω=0；不开火打空 |
| 9.2 | 顺时针、逆时针各转 3 次，每次从静止启动 | spin 判断正确；判定时间 < 1 秒（回放汇总里的 "spin ... decided at"）；ω 钉在 ±2.51 |
| 9.3 | 转动中录 2 分钟 `record.sh aim outpost`，回放 | 三块板 z 先验残差均值都 < 2 cm；三块板都出现在调试图里且不是灰色 |
| 9.4 | 打转动前哨 100 发（5、7 m） | 命中率记录；目标 ≥ 80% |
| 9.5 | 转动中途停转（模拟 3 分钟停转） | 两个投票窗口内改判为 stopped，之后不再打空 |
| 9.6 | 前哨被遮挡 0.5 秒后重新出现 | 不重新初始化（id 不变），预测框接得上 |

三块板高度错位（z 残差有一块明显偏离）：先看是否误把别的板当成同一块（association_ambiguous 多不多），再看 outpost.min_plate_hits 和 tracking.position_gate_m。

## 10. 稳定性和比赛流程

| # | 步骤 | 通过标准 |
|---|---|---|
| 10.1 | `systemctl --user enable --now bof-vision`，断电重启整车 | 开机后自动起来，日志在 artifacts/logs/ |
| 10.2 | 运行中拔相机 5 秒再插回 | 相机节点自动恢复；aim 重新跟踪 |
| 10.3 | 不装弹，运行中拔 C 板 USB 再插回 | 视觉 3 秒内恢复。拔出期间看云台：固件加超时之前，云台会保持最后一帧命令（已知问题，见第 0 节第 7 条）；加了之后应回到遥控/扫描 |
| 10.4 | 不装弹，`kill -9` 掉 pipeline_node、再 kill -9 aim_node | 都在 1 秒后重启；aim_node 被杀期间云台保持最后一帧（同上） |
| 10.5 | 和雷达、导航一起连续运行 15 分钟 | 不崩溃；`/diagnostics` 的 command_loop_overruns 增长 < 1%（500 Hz 命令线程）；CPU 温度不降频 |
| 10.6 | 比赛配置 match.yaml 下复测 8.3 一次 | 与调试配置结果一致 |

## 11. 记录

每个阶段建一个目录 `artifacts/field_tests/<日期>_<阶段>/`，放：当时的 sentry.yaml 副本、bag 名、下面这张表、遇到的问题。bag 和日志按 artifacts/ 默认位置保存，只记路径。

| 项 | 记录 |
|---|---|
| 日期 / 场地 / 车号 | |
| 遥控 s[0]、s[1] 对应的物理拨杆 | |
| 检测模型 / 曝光 | |
| 相机内参 RMS | |
| 测距误差 2/4/6 m | |
| 手眼 RMS / 板位置散布 | |
| camera_latency_s | |
| 云台滞后 / 发弹延迟 | |
| control_delay_s（低速 / 高速） | |
| yaw / pitch 偏置 | |
| 实测弹速 | |
| 静止 2/5/8 m 命中率 | |
| 小陀螺低速 / 高速命中率 | |
| 前哨转向判定时间 / 三板 z 残差 / 命中率 | |
| CPU 占用（视觉 / 总） | |

## 12. 停止条件和找谁

| 现象 | 处理 |
|---|---|
| 云台方向反、乱甩、不受遥控 | s[0] 下，记录现象，找电控核对安装方向和固件 |
| 串口 CRC 错误持续增长 | 查 USB 线和供电；请电控确认帧格式（开发交接第 5 节） |
| 视觉帧被错位解析导致角度跳变 | 固件需要对视觉帧做 CRC 校验后再用（开发交接第 5 节第 1 项） |
| 拔线或视觉崩溃后云台仍按旧命令转、仍在打 | s[0] 下；固件需要视觉指令超时（开发交接第 5 节第 5 项） |
| 检测在某距离、某兵种不稳 | 记录 bag（record.sh images），作为模型训练数据 |
| 命中率低但调试图对得上 | 查弹速、control_delay_s、偏置；必要时考虑空气阻力 |
| 导航掉帧 | 绑核、降检测分辨率或换 GPU 推理，先保导航 |

## 13. 验收指标（参考方案 v1）

| 项目 | 目标 | 在哪测 |
|---|---|---|
| 前哨命中率 | ≥ 80% | 9.4 |
| 前哨转向判断 | 旋转场景正确率 > 95%，静止误判 < 5% | 9.1、9.2 |
| 转向收敛 | < 1 s | 9.2 |
| 普通车 | 跟踪连续，无明显切板抖动 | 8.2、8.3 |
| 命令线程 | 500 Hz，超时 < 1% | 10.5 |
| 视觉 CPU | 不挤占雷达和导航，无掉帧 | 3.4、10.5 |
