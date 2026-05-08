# 机器人 STL 外观模型说明

本目录用于存放 URDF 引用的 STL 外观模型。由于 STL 文件体积较大，仓库默认不提交这些二进制文件。

请从 GitHub Releases 下载：

```text
carnation_description_meshes.zip
```

在仓库根目录解压后，应得到以下文件：

```text
src/carnation_description/meshes/base_base_link.stl
src/carnation_description/meshes/base_link.stl
src/carnation_description/meshes/lower_arm_link.stl
src/carnation_description/meshes/lower_lift_link.stl
src/carnation_description/meshes/upper_arm_link.stl
src/carnation_description/meshes/upper_lift_link.stl
```

如果缺少这些文件，RViz 中机器人外观模型会显示不完整。
