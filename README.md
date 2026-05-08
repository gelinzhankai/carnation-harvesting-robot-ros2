# 康乃馨采收机器人 ROS2 仿真系统

本仓库是一个基于 ROS2 Jazzy 的康乃馨采收机器人仿真项目，用于演示从视觉识别、目标定位到采收执行机构分阶段运动控制的闭环流程。

```text
图像输入 -> YOLOv8m2 康乃馨检测 -> 像素中心 -> 世界坐标目标点 -> 分阶段采收动作仿真
```

项目主要面向毕业设计、论文验证和机器人软件流程演示。当前版本为软件仿真系统，不直接面向真实硬件部署。

## 功能概览

| 模块 | 功能 |
|---|---|
| `carnation_description` | 机器人 URDF、STL 外观模型、RViz 配置 |
| `carnation_interfaces` | 自定义花朵检测消息 `FlowerDetection` |
| `carnation_perception` | 图像源发布、YOLOv8m2 推理、检测框标注 |
| `carnation_localization` | 像素坐标到相机坐标/世界坐标的转换 |
| `carnation_planning` | 按部件顺序执行采收动作的仿真控制 |
| `tools` | Windows 屏幕区域识别、截图桥接、双目演示工具 |

## 环境要求

| 项目 | 推荐环境 |
|---|---|
| 操作系统 | WSL2 Ubuntu 24.04 |
| ROS | ROS2 Jazzy |
| 可视化 | RViz2、rqt_image_view |
| Python | Python 3.12 |
| 视觉推理 | OpenCV、cv_bridge、Ultralytics YOLOv8 |
| 模型描述 | URDF + STL |

## 克隆仓库

为了直接复用仓库中已有 launch 默认路径，建议将仓库克隆到 `/root/carnation_harvest`：

```bash
cd /root
git clone https://github.com/gelinzhankai/carnation-harvesting-robot-ros2.git carnation_harvest
cd /root/carnation_harvest
```

如果克隆到其他目录，需要在运行 launch 时显式传入 `model_path:=...`，或同步修改配置文件中的默认路径。

## 获取外部资源

仓库不直接提交大体积二进制资源。复现前需要从 GitHub Releases 下载 STL 外观模型包和 YOLO 权重。

### 1. 下载 STL 外观模型

请从 Releases 下载：

```text
carnation_description_meshes.zip
```

在仓库根目录解压：

```bash
cd /root/carnation_harvest
unzip /path/to/carnation_description_meshes.zip -d /root/carnation_harvest
```

解压后应存在：

```text
/root/carnation_harvest/src/carnation_description/meshes/base_base_link.stl
/root/carnation_harvest/src/carnation_description/meshes/base_link.stl
/root/carnation_harvest/src/carnation_description/meshes/lower_arm_link.stl
/root/carnation_harvest/src/carnation_description/meshes/lower_lift_link.stl
/root/carnation_harvest/src/carnation_description/meshes/upper_arm_link.stl
/root/carnation_harvest/src/carnation_description/meshes/upper_lift_link.stl
```

缺少 STL 文件时，RViz 中机器人外观会显示不完整。

### 2. 下载 YOLO 权重

请从 Releases 下载：

```text
carnation_yolov8m2_best.pt
```

下载后放入 WSL 工作区：

```bash
mkdir -p /root/carnation_harvest/models
cp /path/to/carnation_yolov8m2_best.pt /root/carnation_harvest/models/
```

最终应存在以下文件：

```text
/root/carnation_harvest/models/carnation_yolov8m2_best.pt
```

如需运行 Windows 屏幕桥接演示，还需要在 Windows 侧准备同一份权重：

```text
H:\carnation_detection\carnation_yolov8m2_best.pt
```

## 安装依赖

安装 ROS2 相关依赖：

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

如果系统提示不允许直接使用 `pip` 写入系统 Python，建议创建虚拟环境或按 Ubuntu 提示添加 `--break-system-packages`。

## 编译工作区

```bash
cd /root/carnation_harvest
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

重新打开终端后，需要重新加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source /root/carnation_harvest/install/setup.bash
```

## 快速复现：静态图片采收仿真

该模式不依赖 Windows 屏幕桥接，适合首次验证 ROS2 节点、YOLO 推理、坐标转换和采收动作是否能够完整跑通。

```bash
cd /root/carnation_harvest
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch carnation_localization vision_to_target_pose.launch.py \
  source_type:=image_directory \
  source_path:=/root/carnation_harvest/data/apply_images \
  sequence_detections:=false \
  loop_images:=true \
  use_image_view:=true \
  use_rviz:=true \
  use_planning:=true
```

正常运行后应看到：

| 窗口 | 现象 |
|---|---|
| RViz2 | 机器人模型、目标点、采收机构运动 |
| rqt_image_view | 当前图像和正在执行采收的目标检测框 |
| 终端 | 目标像素坐标、相机坐标和世界坐标日志 |

## 完整演示：Windows 屏幕识别到 ROS2 采收仿真

该模式用于复现最终演示流程。Windows 侧负责实时显示 YOLO 检测画面，ROS2 侧在每次需要新目标时触发截图桥接，并基于截图完成采收规划。

启动顺序必须为：

```text
1. Windows 侧 YOLO 屏幕区域识别
2. Windows 侧截图桥接
3. WSL/ROS2 一体化 launch
```

### 1. 准备 Windows 侧 Python 环境

示例路径使用 `H:\carnation_detection`。如果使用其他路径，需要同步修改 `tools/*.bat` 中的路径。

```bat
mkdir H:\carnation_detection
python -m venv H:\carnation_detection\.venv
H:\carnation_detection\.venv\Scripts\python.exe -m pip install opencv-python pillow numpy ultralytics
```

将权重放入：

```text
H:\carnation_detection\carnation_yolov8m2_best.pt
```

### 2. 启动 Windows 侧 YOLO 屏幕区域识别

在 Windows 资源管理器中打开：

```text
\\wsl.localhost\Ubuntu-24.04\root\carnation_harvest\tools
```

双击运行：

```text
start_detect_yolov8m2_screen_region.bat
```

运行后按提示输入屏幕区域、置信度阈值和图像尺寸。脚本会写入配置文件：

```text
H:\carnation_detection\ros_screen_region_config.json
```

配置文件示例：

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

### 3. 启动 Windows 到 ROS 的截图桥接

保持第 2 步窗口运行，再双击：

```text
start_ros_screen_region_frame_bridge.bat
```

桥接脚本会等待 ROS2 触发信号，并将截图写入：

```text
H:\carnation_detection\ros_screen_region_frame.jpg
```

当桥接成功截图时，窗口会输出类似内容：

```text
Triggered capture: left=..., top=..., right=..., bottom=... -> H:\carnation_detection\ros_screen_region_frame.jpg
```

### 4. 启动 ROS2 一体化 launch

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

## 节点说明

| 节点 | 功能 |
|---|---|
| `image_source_node` | 发布图像源，可读取静态图片目录或 Windows 桥接截图 |
| `yolo_detector_node` | 加载 YOLOv8m2 权重，输出检测框、置信度、像素中心和标注图 |
| `pixel_to_world_node` | 将花朵像素中心转换为相机坐标和世界坐标，并发布 `/target_pose` |
| `robot_state_publisher` | 根据 URDF 发布机器人 TF |
| `simple_target_follower` | 按采收流程发布关节状态，驱动 RViz 中的仿真动作 |
| `rqt_image_view` | 显示当前采收目标的标注图 |
| `rviz2` | 显示机器人模型、TF、目标点和采收动作 |

## 主要话题

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | YOLO 输入图像 |
| `/carnation/detections/image_annotated` | `sensor_msgs/Image` | 带检测框的标注图 |
| `/carnation/pixel_center` | `carnation_interfaces/FlowerDetection` | 类别、置信度、像素中心和检测框 |
| `/carnation/target_point_camera` | `geometry_msgs/PointStamped` | 相机坐标系下的目标点 |
| `/carnation/target_point_world` | `geometry_msgs/PointStamped` | 世界坐标系下的目标点 |
| `/target_pose` | `geometry_msgs/PoseStamped` | 采收规划节点接收的目标位姿 |
| `/joint_states` | `sensor_msgs/JointState` | 仿真关节状态 |
| `/harvest_cycle_busy` | `std_msgs/Bool` | 采收周期忙碌/空闲状态 |
| `/carnation/target_rejected` | `std_msgs/Bool` | 当前目标被定位层拒绝 |
| `/carnation/frame_exhausted` | `std_msgs/Bool` | 当前帧无可执行目标 |

## 常用调试命令

检查图像话题：

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /carnation/detections/image_annotated
```

查看检测输出：

```bash
ros2 topic echo /carnation/pixel_center --once
```

查看坐标转换结果：

```bash
ros2 topic echo /target_pose --once
ros2 topic echo /carnation/target_point_world --once
```

查看采收周期状态：

```bash
ros2 topic echo /harvest_cycle_busy
```

清理旧 ROS2 进程：

```bash
pkill -f "ros2 launch"
pkill -f "image_source_node|yolo_detector_node|pixel_to_world_node|simple_target_follower|robot_state_publisher|rqt_image_view|rviz2"
ros2 daemon stop
ros2 daemon start
```

## 参数位置

| 文件 | 说明 |
|---|---|
| `src/carnation_perception/config/yolo_detector.yaml` | YOLO 权重路径、置信度阈值、目标选择逻辑 |
| `src/carnation_perception/config/image_source.yaml` | 图像源、触发截图、图片循环参数 |
| `src/carnation_localization/config/pixel_to_world.yaml` | 单目简化深度、目标偏置、工作空间范围 |
| `src/carnation_localization/config/stereo_pixel_to_world.yaml` | 双目基线、相机内参、论文坐标转换矩阵 |
| `src/carnation_planning/config/simple_target_follower.yaml` | 各关节速度、等待时间、采收动作顺序 |
| `src/carnation_description/urdf/carnation_robot_v0.urdf` | 机器人结构、STL 模型、相机坐标系 |

## 注意事项

- 当前坐标转换以简化仿真为主，默认通过目标框高度和固定深度模型估计工作空间内的目标点。
- 仓库不包含原始训练数据集和 YOLO 权重文件，权重需从 Releases 下载。
- `build/`、`install/`、`log/`、模型权重、运行缓存和视频文件不应提交到 Git。
- Windows 屏幕桥接脚本中的 `H:\carnation_detection` 是默认示例路径，其他机器可按实际路径修改 bat 文件。
- 如果 RViz 中没有模型或 TF，优先确认已经执行 `source install/setup.bash`，并确认 launch 中包含 `robot_state_publisher`。
