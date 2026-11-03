# 模型说明

这里的 9 个固定版本模型文件（约 21.4 MiB）随 Git 仓库及 `src/scripts/dev/package.sh` 生成的源码包保存，克隆后无需从其他本地项目复制。使用普通 Git 二进制文件，不依赖 Git LFS 或子模块。`SHA256SUMS` 记录每个文件的 SHA-256；在仓库根目录运行 `python3 src/scripts/dev/check_repo.py` 或 `sha256sum -c src/models/SHA256SUMS` 核验。

下表导入脚本用于从原始来源恢复相同版本，仍固定 SHA-256，防止把同名的其他模型按错误的格式解析。来源和许可说明保持不变。

| 目录 | 文件 | 导入方式 | 来源 |
|---|---|---|---|
| tongji_yolov5 | yolov5.xml/.bin | `src/scripts/dev/import_model.sh /path/to/sp_vision_25` | 同济 sp_vision_25 assets |
| tongji_yolov8 | yolov8.xml/.bin、tiny_resnet.onnx | 同上 | 同上 |
| tongji_yolo11 | yolo11.xml/.bin | 同上 | 同上 |
| shenzhen | shenzhen-0526.onnx、shenzhen-0708.onnx | `src/scripts/dev/import_shenzhen_models.sh /path/to/rmcs_auto_aim_v2` | 用户提供的南理工仓库（解析格式参考 Alliance models/shenzhen.hpp） |

sp_vision_25 本地 HEAD：bd9f5e798fa3c6dd3b483ae6627796afb41c608d，仓库许可证为 MIT。它的 readme 参考文献还提到深圳和北京科技大学的识别模型。各权重的训练来源和许可链没有单独审定；这里只是本地评估用的副本，不声称是同济原创模型，也不是已核准可对外分发的资产。

## 格式

| 后端 | 输入 | 输出 | 类别 |
|---|---|---|---|
| tongji_yolov5 | 640 | 1×N×22 | 9 类；第 8 类为非装甲，丢弃 |
| shenzhen_0526/0708 | 640 | 1×N×22 | 第 7/8 类为小/大基地（不能按同济的第 8 类过滤） |
| tongji_yolov8 | 416 | 1×14×3549 | 只分红蓝；数字由 tiny_resnet 分类（9 类含 not_armor，阈值 classifier_confidence，同济默认 0.8） |
| tongji_yolo11 | 640 | 1×50×8400 | 38 类，按同济 armor_properties 映射颜色、兵种、大小板 |

YOLO11 只采用基地和 3/4/5 号步兵的大小板信息。上游把英雄标为小板，但英雄是大装甲板，照搬会让 PnP 用错尺寸；sp25 自己的 YOLO11 代码也因此会把英雄检测全部丢掉。

## 评估结果

没有人工标注真值时，候选数量和耗时都不代表检测精度。

- sp25 demo.avi（蓝方，300 帧，CPU）：YOLOv5 279 帧有候选，YOLO11 260 帧，YOLOv8 66 帧（分类器阈值降到 0.01 时 108 帧）。推理耗时 p50 约 3.1 / 5.2 / 3.4 ms。YOLOv8 检出少，主要被分类器置信度过滤掉。证据在 log/artifacts/output/yolo_v8_v11_20261102。
- 深圳 0526/0708 在 RPS 前哨视频片段上：0526 输出前哨标签，0708 输出哨兵标签，存在类别误判。
- 这些都是旧赛季的模型。2027 的装甲板和前哨外观要用本队录的数据评估，必要时采集标注、重新训练。

只支持上表这些输出格式，别的 YOLO 权重不能直接放进来用。OpenVINO 的 GPU 设备指 Intel 核显，不是 NVIDIA TensorRT。
