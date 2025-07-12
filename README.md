# ROS1 手眼标定项目

> **参考教程**: [手眼标定】ros / easy_handeye + ur5 + realsense d435i](https://zhuanlan.zhihu.com/p/92339362)
> 
> ⚠️ **注意**: 原教程中的部分仓库和launch文件已过时，仅可参考流程和节点架构。本仓库已对相关包的配置进行了更新和优化。

## 项目概述

由于ROS2的手眼标定仓库支持不够完善，本项目采用ROS1 Noetic版本进行手眼标定。以下是详细的实施步骤和关键要点：

## 环境准备

### 1. 依赖安装
按照参考教程克隆相关仓库，确保使用 `noetic` 或 `noetic-devel` 分支：
```bash
# 克隆仓库后会自动编译
```

### 2. UR机器人标定参数导出
使用UR机器人的标定参数工具导出标定参数：
```bash
roslaunch ur_calibration calibration_correction.launch \
    robot_ip:=169.254.100.182 \
    target_filename:="/catkin_ws/src/ur5_robot_gripper/my_robot_calibration.yaml"
```

### 3. 标定板准备
- 访问 [Online ArUco markers generator](https://chev.me/arucogen/) 下载标定板文件
- **重要**: 确保下载的编号和尺寸与后续launch文件中的配置一致
- 尺寸指的是整个黑色方形区域的宽度
- 可选择打印或使用平板显示（确保显示尺寸准确）

## 标定执行

### 启动标定程序
```bash
roslaunch ur5_robot_gripper hand_eye_calibration.launch
```

### 界面操作
- 系统会显示RViz窗口（可能提示2025年5月31日停止支持，可忽略）
- 建议开启TF显示，查看 `tool0` 和 `base_link` 的变换关系

### 坐标系选择说明
经过测试验证：
- **基座坐标系**: 使用 `base_link`
- **末端坐标系**: 可选择 `tool0` 或 `tool0_controller`，两者均可正常工作
- **推荐使用**: `tool0`（使用频率更高）

## 标定策略

实际标定过程中，可以不依赖easy_handeye提供的MoveIt运动规划功能。建议：
- 手动移动机器人到多个姿态
- 确保每个姿态下marker清晰可见，最好足够近，因为realsense相机分辨率比较差
- 选择变换幅度足够大的姿态进行标定

## 标定结果

本项目的最终标定参数如下：

```yaml
translation: 
  x: -0.03265645673390799
  y: -0.10103297404441841
  z: 0.04246950147068641
rotation: 
  x: 0.013022926649812893
  y: -0.0013546827673798223
  z: -0.001520683101406884
  w: 0.9999131240957778
```

✅ **验证结果**: 标定参数与CAD模型设计基本吻合。