# BOF Vision

RoboMaster 哨兵视觉与自瞄工程：单相机检测装甲板，击打前哨站和敌方机器人。ROS 2 Jazzy / C++17 / colcon。

## 现状（2026-11-02）

**当前处于实机集成验证准备阶段。电控源码已修改，等待电控烧录；完整目标编译结果、实际烧录版本和板上验证记录尚待提供。**

| 部分 | 状态 |
|---|---|
| 检测、PnP、像素关联、录像、内参标定 | 已实现；海康实拍未验收 |
| 自瞄（EKF 跟踪、前哨转向判断、选板、弹道、开火判据、500 Hz 命令） | 已实现，默认关闭；仿真验证通过，未上车 |
| 云台串口 | 已有 cboard/SP 接入；真实串口与安装约定未验收 |
| 电控停止保护 | 接收校验、通信过期锁定、停止输出源码已修改；40 项主机隔离检查通过，等待烧录及板上验证 |
| 上车工具 | 自瞄调试画面、手眼标定、测延迟、bag 回放、比赛自启、录包脚本 |
| 导航/决策 | 默认不接入，自瞄独立运行；接口保留 |

已验证的内容包括 ROS 合成图像链路、CLI/ROS 配置一致性和 12 帧无窗口运行。仿真、离线和主机替身检查均不代表实机效果，完整系统尚未验收。

下一步先由电控完成编译确认和烧录，再在执行器断能或隔离环境核对停止链路与实际输出。**停止保护完成板上安全验证前，暂停装弹测试。** [原实车测试方案](docs/TEST_PLAN.md)尚未整体按新增保护行为复核，当前状态以[开发交接](docs/DEVELOPMENT.md)为准。

## 目录

```text
BOF_vision_27/
├── src/
│   ├── bof_interfaces/   # ROS 消息
│   ├── bof_vision/       # 算法核心（core/）、ROS 节点（src/）、现场工具（apps/）、检查（test/）
│   └── bof_bringup/      # launch 和配置（sentry.yaml 调参，match.yaml 比赛）
├── scripts/              # 构建、环境、现场窗口、比赛启动、录包
│   ├── system/           # udev 规则、systemd 服务、自启安装
│   └── dev/              # 模型导入、评估、检查和打包
├── docs/                 # 使用说明、开发交接、测试方案与电控修改说明
├── models/               # 模型权重（不进源码包）和来源说明
├── third_party/          # 开源来源声明和许可证
└── artifacts/            # 运行证据、日志、bag（不进源码包）
```

平时只改三处：参数 `src/bof_bringup/config/sentry.yaml`，算法 `src/bof_vision/`，启动 `src/bof_bringup/launch/vision.launch.py`。

## 快速开始

```bash
cd ~/BOF_vision_27
bash scripts/build.sh                     # 无设备演示版（不含 OpenVINO/海康）
source scripts/env.sh
bash scripts/run_live.sh --synthetic --yolo light_pair
```

实际相机和神经网络需要启用相应的 OpenVINO/MVS 后端，见[使用说明](docs/USAGE.md)第 1 节。`build.sh` 每次会将未显式启用的后端设为 OFF；关闭 MVS 或现场工具时会清理对应旧程序，避免运行到历史构建产物。

## 电控交接与调试资料

固件在独立目录 `/home/lmy/桌面/sentry_gimbol_new./sentry_gimbol_new/`，不包含在本视觉工程的源码包中。

- [电控修改记录与验证边界](/home/lmy/桌面/sentry_gimbol_new./sentry_gimbol_new/docs/视觉通信与停止保护_修改记录.txt)
- [40 项主机检查结果与源码哈希](/home/lmy/桌面/sentry_gimbol_new./sentry_gimbol_new/docs/vision_safety_host_results.txt)

新增保护在启动、通信失效或无效消息后锁定视觉控制，恢复连接不会自动恢复旧指令。解除条件、RC 手动路径的边界及暂定超时值见[开发交接第 5 节](docs/DEVELOPMENT.md#5-云台串口协议)。这不是整机断能或独立硬件急停。

开始排查时，先保存最小资料包：**实际运行配置及有效快照、完整启动日志、一段原始图像记录、具体异常及发生时间**。同时记录视觉/固件版本；硬件详情、标定资料、时间戳和资源占用的完整清单见[开发交接第 10 节](docs/DEVELOPMENT.md#10-下一步与所需资料)。

## 文档

- [使用说明](docs/USAGE.md)：编译、启动、调参、标定、上车调试、比赛启动、录包、排查。
- [开发交接](docs/DEVELOPMENT.md)：当前状态、代码分工、电控修改、验证边界和下一步资料清单。
- [实车测试方案](docs/TEST_PLAN.md)：原分阶段方案及记录表；已补状态提示，正文仍待按新保护行为整体复核。
- [电控修改需求说明（TXT）](docs/电控端安全停止与接收校验修改说明.txt)：修改前问题、源码证据和停止链路验收要求。
- [旧固件修改建议](docs/FIRMWARE_CHANGES.md)：历史参考，示例不能覆盖当前固件实现。
- [模型说明](models/README.md)：各模型格式、来源与评估结果。
- [来源声明](third_party/NOTICE.md)：同济 sp_vision_25、RPS auto_aim_rps26 等 MIT 代码的具体改编。

历史备份在项目外的 `~/BOF_vision_27_archive/`。
