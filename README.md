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

下一步先由电控完成编译确认和烧录，再在执行器断能或隔离环境核对停止链路与实际输出。**停止保护完成板上安全验证前，暂停装弹测试。** [原实车测试方案](src/docs/TEST_PLAN.md)尚未整体按新增保护行为复核，当前状态以[开发交接](src/docs/DEVELOPMENT.md)为准。

## 目录

```text
BOF_vision_27/
├── README.md             # 项目首页
├── src/
│   ├── bof_interfaces/   # ROS 消息
│   ├── bof_vision/       # core/ 算法、src/ 节点、apps/ 工具、test/ 检查
│   ├── bof_bringup/      # launch 和配置
│   ├── scripts/          # 构建、运行、检查、打包和系统服务
│   ├── models/           # 模型、校验清单与来源说明
│   ├── docs/             # 构建、使用、交接资料
│   ├── third_party/      # 来源声明和许可证
│   └── LICENSE
├── build/                # 编译中间文件；dist/ 保存分发包
├── install/              # colcon 安装结果
└── log/                  # 构建日志；artifacts/ 保存录制和验证输出
```

根目录另有 `.git/`、`.github/`、`.gitignore`、`.gitattributes` 隐藏项，用于 Git/GitHub 管理。以下命令均在工作区根目录执行。

平时只改三处：参数 `src/bof_bringup/config/sentry.yaml`，算法 `src/bof_vision/`，启动 `src/bof_bringup/launch/vision.launch.py`。

## 快速开始

```bash
git clone git@github.com:lmy-lmy-666/BOF_vision_27.git
cd BOF_vision_27
python3 src/scripts/dev/check_repo.py          # 核对模型、配置引用、文档链接
bash src/scripts/build.sh                     # 无设备演示版（不含 OpenVINO/海康）
source src/scripts/env.sh
bash src/scripts/run_live.sh --synthetic --yolo light_pair
```

新机器先按[构建说明](src/docs/构建方式说明.txt)安装依赖。模型已随仓库提供；构建产物需本机重新生成。仓库内容、外部依赖和提交步骤见[仓库与迁移说明](src/docs/REPOSITORY.md)。

实际相机和神经网络需要启用相应的 OpenVINO/MVS 后端，见[使用说明](src/docs/USAGE.md)第 1 节。`build.sh` 每次会将未显式启用的后端设为 OFF；关闭 MVS 或现场工具时会清理对应旧程序，避免运行到历史构建产物。

## 电控交接与调试资料

云台固件 `sentry_gimbol_new` 是独立工程；本仓库保存交接资料快照，固件源码需另行获取。快照来源与边界见[交接资料说明](src/docs/firmware/README.md)。

- [电控修改记录与验证边界](src/docs/firmware/视觉通信与停止保护_修改记录.txt)
- [40 项主机检查结果与源码哈希](src/docs/firmware/vision_safety_host_results.txt)

新增保护在启动、通信失效或无效消息后锁定视觉控制，恢复连接不会自动恢复旧指令。解除条件、RC 手动路径的边界及暂定超时值见[开发交接第 5 节](src/docs/DEVELOPMENT.md#5-云台串口协议)。这不是整机断能或独立硬件急停。

开始排查时，先保存最小资料包：**实际运行配置及有效快照、完整启动日志、一段原始图像记录、具体异常及发生时间**。同时记录视觉/固件版本；硬件详情、标定资料、时间戳和资源占用的完整清单见[开发交接第 10 节](src/docs/DEVELOPMENT.md#10-下一步与所需资料)。

## 文档

- [构建说明](src/docs/构建方式说明.txt)：新机器依赖和编译入口。
- [仓库与迁移说明](src/docs/REPOSITORY.md)：目录职责、Git 提交、完整性检查和打包。

- [使用说明](src/docs/USAGE.md)：编译、启动、调参、标定、上车调试、比赛启动、录包、排查。
- [开发交接](src/docs/DEVELOPMENT.md)：当前状态、代码分工、电控修改、验证边界和下一步资料清单。
- [实车测试方案](src/docs/TEST_PLAN.md)：原分阶段方案及记录表；已补状态提示，正文仍待按新保护行为整体复核。
- [电控修改需求说明（TXT）](src/docs/电控端安全停止与接收校验修改说明.txt)：修改前问题、源码证据和停止链路验收要求。
- [旧固件修改建议](src/docs/FIRMWARE_CHANGES.md)：历史参考，示例不能覆盖当前固件实现。
- [模型说明](src/models/README.md)：各模型格式、来源与评估结果。
- [来源声明](src/third_party/NOTICE.md)：同济 sp_vision_25、RPS auto_aim_rps26 等 MIT 代码的具体改编。

历史备份在项目外的 `~/BOF_vision_27_archive/`。
