# 康乃馨采收机器人 ROS2 仿真系统

本仓库是一个基于 ROS2 的康乃馨采收机器人仿真项目，用于完成从视觉识别到目标定位，再到采收机构分阶段运动控制的闭环验证。最终演示流程为：

```text
屏幕/视频画面 -> YOLOv8m2 目标检测 -> 花朵像素中心 -> 世界坐标目标点 -> 分阶段采收动作仿真
```

本项目主要用于毕业设计、论文验证和仿真演示，不直接面向真实采收机器人部署。

## 运行环境

| 项目 | 版本或说明 |
|---|---|
| 系统 | WSL2 Ubuntu 24.04 |
| ROS | ROS2 Jazzy |
| 可视化 | RViz2、rqt_image_view |
| 机器人模型 | URDF + STL 外观模型 |
| 视觉识别 | Python、OpenCV、cv_bridge、Ultralytics YOLOv8 |
| 坐标转换 | 相机内参、简化深度模型、TF 坐标变换 |
| 运动控制 | 自定义分阶段关节状态仿真节点 |
| Windows 桥接 | Python ImageGrab 屏幕区域截图 |

## 项目结构

```text
carnation_harvest/
├── src/
│   ├── carnation_description/     # URDF、STL 模型、RViz 配置
│   ├── carnation_interfaces/      # 自定义 FlowerDetection 消息
│   ├── carnation_perception/      # 图像源节点、YOLO 检测节点
│   ├── carnation_localization/    # 像素到世界坐标转换、双目坐标转换节点
│   └── carnation_planning/        # 分阶段采收动作仿真节点
├── tools/
│   ├── screen_region_detect_yolov8m2.py
│   ├── start_detect_yolov8m2_screen_region.bat
│   ├── screen_region_frame_bridge.py
│   ├── start_screen_region_frame_bridge.bat
│   └── demo_stereo_from_single_image.py
├── data/apply_images/             # 少量静态演示图片
├── models/                        # 模型权重不提交到 Git
└── README.md
```

## 模型权重说明

训练好的 YOLOv8m2 权重文件属于大体积二进制文件，不建议直接提交到 Git 仓库。

ROS 侧默认权重路径：

```text
/root/carnation_harvest/models/carnation_yolov8m2_best.pt
```

建议在 GitHub Releases 中上传的文件名：

```text
carnation_yolov8m2_best.pt
```

从 GitHub Releases 下载后，放入 WSL 工作区：

```bash
mkdir -p /root/carnation_harvest/models
cp /path/to/carnation_yolov8m2_best.pt /root/carnation_harvest/models/
```

Windows 侧屏幕识别脚本也需要一份权重文件，建议放在：

```text
H:\carnation_detection\carnation_yolov8m2_best.pt
```

开发时使用的原始训练输出路径为：

```text
H:\carnation_detection\runs\carnation_yolov8m2\weights\best.pt
```

## 依赖安装

安装 ROS 相关依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-opencv \
  python3-numpy \
  ros-jazzy-cv-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-rqt-image-view
```

安装 YOLO 推理依赖：

```bash
python3 -m pip install ultralytics
```

Windows 侧桥接脚本需要在对应虚拟环境中安装：

```bat
H:\carnation_detection\.venv\Scripts\python.exe -m pip install opencv-python pillow numpy ultralytics
```

## 编译

```bash
cd /root/carnation_harvest
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 最终演示：Windows 屏幕识别到 ROS2 采收仿真

这是本项目最终集成版本的主要运行方式。

### 1. 启动 Windows 侧 YOLO 屏幕区域识别

如果通过 `\\wsl.localhost` 访问本仓库，可运行：

```bat
\\wsl.localhost\Ubuntu-24.04\root\carnation_harvest\tools\start_detect_yolov8m2_screen_region.bat
```

运行后输入屏幕截取区域、置信度阈值和图像尺寸。脚本会把这些参数写入：

```text
H:\carnation_detection\ros_screen_region_config.json
```

示例：

```json
{
  "left": 1760,
  "top": 600,
  "right": 2560,
  "bottom": 1500,
  "confidence_threshold": 0.42,
  "image_size": 640
}
```

### 2. 启动 Windows 到 ROS 的截图桥接脚本

运行：

```bat
\\wsl.localhost\Ubuntu-24.04\root\carnation_harvest\tools\start_screen_region_frame_bridge.bat
```

该脚本会等待 ROS 触发信号，并将最新截图写入：

```text
H:\carnation_detection\ros_screen_region_frame.jpg
```

每次真正完成截图时，窗口中会输出：

```text
Triggered capture: left=..., top=..., right=..., bottom=... -> H:\carnation_detection\ros_screen_region_frame.jpg
```

### 3. 启动 ROS2 一体化 launch

```bash
cd /root/carnation_harvest
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch carnation_localization vision_to_target_pose.launch.py \
  source_type:=triggered_latest_image \
  source_path:=/mnt/h/carnation_detection/ros_screen_region_frame.jpg \
  trigger_file_path:=/mnt/h/carnation_detection/ros_screen_region_trigger.txt \
  sequence_detections:=false \
  use_image_view:=true \
  use_rviz:=true \
  use_planning:=true
```

该 launch 会启动：

| 节点 | 功能 |
|---|---|
| `image_source_node` | 读取 Windows 桥接截图并发布 `/camera/image_raw` |
| `yolo_detector_node` | 执行 YOLOv8m2 推理，发布检测结果和标注图 |
| `pixel_to_world_node` | 将像素中心转换为相机坐标和世界坐标，并发布 `/target_pose` |
| `robot_state_publisher` | 发布机器人 URDF 对应的 TF |
| `simple_target_follower` | 根据目标点执行分阶段采收动作 |
| `rqt_image_view` | 显示当前检测标注图 |
| `rviz2` | 显示机器人模型、目标点和运动过程 |

## 静态图片演示模式

如果不使用 Windows 屏幕桥接，也可以直接使用仓库中的静态图片目录：

```bash
ros2 launch carnation_localization vision_to_target_pose.launch.py \
  source_type:=image_directory \
  source_path:=/root/carnation_harvest/data/apply_images \
  sequence_detections:=false \
  loop_images:=true \
  use_image_view:=true \
  use_rviz:=true \
  use_planning:=true
```

## 常用调试命令

检查图像是否发布：

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /carnation/detections/image_annotated
```

检查 YOLO 检测输出：

```bash
ros2 topic echo /carnation/pixel_center --once
```

检查坐标转换输出：

```bash
ros2 topic echo /target_pose --once
ros2 topic echo /carnation/target_point_world --once
```

检查采收周期状态：

```bash
ros2 topic echo /harvest_cycle_busy
```

重新运行前清理旧进程：

```bash
pkill -f "ros2 launch"
pkill -f "image_source_node|yolo_detector_node|pixel_to_world_node|simple_target_follower|robot_state_publisher|rqt_image_view|rviz2"
ros2 daemon stop
ros2 daemon start
```

## 主要话题

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | YOLO 输入图像 |
| `/carnation/detections/image_annotated` | `sensor_msgs/Image` | 带检测框的标注图 |
| `/carnation/pixel_center` | `carnation_interfaces/FlowerDetection` | 目标类别、置信度、像素中心和检测框 |
| `/carnation/target_point_camera` | `geometry_msgs/PointStamped` | 相机坐标系下的目标点 |
| `/carnation/target_point_world` | `geometry_msgs/PointStamped` | 世界坐标系下的目标点 |
| `/target_pose` | `geometry_msgs/PoseStamped` | 规划节点接收的目标位姿 |
| `/joint_states` | `sensor_msgs/JointState` | 仿真关节状态 |
| `/harvest_cycle_busy` | `std_msgs/Bool` | 采收周期忙碌/空闲状态 |
| `/carnation/target_rejected` | `std_msgs/Bool` | 当前目标被定位层拒绝 |
| `/carnation/frame_exhausted` | `std_msgs/Bool` | 当前帧无可执行目标 |

## 说明

- 当前坐标转换采用简化深度模型，参数位于 `src/carnation_localization/config/pixel_to_world.yaml`。
- 分阶段采收动作参数位于 `src/carnation_planning/config/simple_target_follower.yaml`。
- 机器人结构和相机安装关系位于 `src/carnation_description/urdf/carnation_robot_v0.urdf`。
- YOLO 节点默认固定主点，不再把主点移动到上一采收目标中心。
- 仓库不应提交模型权重、原始训练数据集、`build/install/log`、视频文件和运行缓存。
