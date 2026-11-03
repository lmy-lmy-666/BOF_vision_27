# 仓库结构与迁移

2026-11-03 整理。工作区根目录保留 `README.md` 和四个可见目录；以下命令均在工作区根目录执行。

```text
BOF_vision_27/
├── README.md
├── src/
│   ├── bof_interfaces/
│   ├── bof_vision/
│   ├── bof_bringup/
│   ├── scripts/
│   ├── models/
│   ├── docs/
│   ├── third_party/
│   └── LICENSE
├── build/
├── install/
└── log/
```

Git 元数据 `.git/`、忽略规则 `.gitignore`、文件属性 `.gitattributes` 和 GitHub 配置 `.github/` 仍留在根目录，均为隐藏项。GitHub 首页直接展示根目录的 [README.md](../../README.md)，其中链接到 `src/` 下的文档。克隆时三个生成目录可以不存在，第一次构建会自动创建。

## 目录职责

| 路径 | 内容 | 是否提交 |
|---|---|---|
| `src/bof_interfaces/` | ROS 消息定义 | 是 |
| `src/bof_vision/` | `core/` 算法、`src/` 节点、`apps/` 工具、`test/` 检查 | 是 |
| `src/bof_bringup/` | 配置与 launch | 是 |
| `src/models/` | 9 个模型、SHA-256 清单及来源说明 | 是，普通 Git 文件 |
| `src/scripts/` | 构建、环境、运行；`dev/` 开发工具；`system/` 系统服务 | 是 |
| `src/docs/` | 构建、使用和交接文档；`firmware/` 为交接记录快照 | 是 |
| `src/third_party/` | 来源声明和许可证原文 | 是 |
| `build/` | colcon 编译中间文件；`dist/` 为源码分发包 | 否 |
| `install/` | colcon 安装结果 | 否 |
| `log/` | colcon 日志；`artifacts/` 为运行日志、录制和验证输出 | 否 |
| `src/data/` | 可选的本机数据集、标定输入 | 否，需要时另行交接 |

模型路径相对于 YAML 文件所在目录解析，现在是 `../../models/...`。包目录的位置不变。构建入口使用 `--base-paths src`，避免发现历史输出中的其他工程。

## 常用入口

```bash
python3 src/scripts/dev/check_repo.py
bash src/scripts/build.sh
source src/scripts/env.sh
bash src/scripts/run_live.sh --synthetic --yolo light_pair --headless --frames 12
bash src/scripts/dev/package.sh
```

构建、环境、运行和打包脚本均根据自身位置定位工作区，不要求调用者当前处于工作区。原有 `artifacts/` 已整体迁到 `log/artifacts/`，历史记录没有删除；旧分发包仍可在 `log/artifacts/dist/` 找到，新分发包统一写到 `build/dist/`。

原来安装过 systemd 自启的机器需要重新执行 `bash src/scripts/system/install_autostart.sh` 更新脚本位置。本次目录整理不会自动修改已安装的系统服务。

## 提交到 GitHub

```bash
python3 src/scripts/dev/check_repo.py
git add -A
python3 src/scripts/dev/check_repo.py --require-tracked
git diff --cached --stat
git commit -m "Move project resources under src"
git push
```

普通检查覆盖工作区中已跟踪及未忽略的新文件；`--require-tracked` 额外确认这些文件已经加入 Git，避免漏掉新增文件。检查器验证根目录布局、模型哈希、配置资源引用和文档链接，不会自动提交或推送。GitHub 工作流从实际提交重新检查并验证打包结果。

9 个模型合计约 21.4 MiB，随仓库提供，无需 LFS 或子模块。模型 XML 保留原始换行，避免 Git 自动转换导致哈希变化。来源和许可声明见[模型说明](../models/README.md)。

## 离线迁移与外部依赖

```bash
bash src/scripts/dev/package.sh
cd build/dist
sha256sum -c BOF_vision_27-source.tar.gz.sha256
```

分发包包含 `README.md`、`src/` 和根目录中的 Git/GitHub 配置，不包含 `.git` 历史和三个生成目录。解压后可运行同一个检查器并再次打包。迁移到新机器后重新构建，不能直接复制 symlink-install 的安装结果。

ROS 2 Jazzy、OpenCV、Eigen、yaml-cpp 等系统依赖按[构建说明](构建方式说明.txt)安装。可选 OpenVINO、MVS SDK 通过 `OpenVINO_DIR`、`MVS_ROOT` 指定独立安装位置。云台固件是独立工程，本仓库只包含[交接记录快照](firmware/README.md)。录制、标定及历史验证输出需要时单独交接。
