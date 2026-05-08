# 模型权重说明

本目录用于存放 YOLOv8m2 模型权重，但大体积模型文件不提交到 Git 仓库。

最终演示默认使用的权重文件名：

```text
carnation_yolov8m2_best.pt
```

下载 GitHub Releases 中的权重后，将其放置到：

```text
/root/carnation_harvest/models/carnation_yolov8m2_best.pt
```

ROS launch 默认读取该路径：

```text
/root/carnation_harvest/models/carnation_yolov8m2_best.pt
```

Windows 屏幕识别脚本也需要一份权重文件，建议放置到：

```text
\carnation_detection\carnation_yolov8m2_best.pt
```
