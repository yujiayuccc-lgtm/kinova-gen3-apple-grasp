# Kinova Gen3 Autonomous Apple Grasp

基于 ROS 2、MoveIt 2 和 RGB-D 视觉的 Kinova Gen3 自动苹果搜索、定位、抓取与放置系统。

该项目运行于真实机械臂，能够依次访问预设观察位，从 Gen3 相机获取彩色与深度数据，估计苹果在机械臂基座坐标系中的三维位置，规划无碰撞接近路径，并通过 Robotiq 2F-85 夹爪完成抓取验证和放置。

> 当前版本面向固定工作台上的红色苹果，是一个带安全约束的实机闭环抓取原型，而不是通用果园采摘系统。

## 功能

- 多观察位主动搜索：中心、右侧、左侧、近处、扩展近处和远处
- 从 Gen3 RGB-D RTSP 数据流获取彩色图与深度图
- HSV 颜色分割、目标点云提取和球面拟合
- 三帧稳定检测与图像边缘检查
- 相机坐标到 `base_link` 的实时坐标转换
- MoveIt 2 碰撞场景与笛卡尔路径规划
- 根据目标偏移自动选择垂直接近或折线路径
- Robotiq 2F-85 力度受限闭合和接触验证
- 抓取后验证抬升、放置、释放和自动复位
- 工作空间、目标尺寸、点云数量及拟合误差安全检查
- 默认拒绝真实运动，必须通过双重授权才能执行

## 系统流程

```text
检查硬件与故障状态
        |
        v
依次访问预设观察位 ---- 未检测到目标 ---> 下一个观察位
        |
        v
采集 RGB-D 数据（3 帧）
        |
        v
红色分割与目标点云提取
        |
        v
球面拟合与基座坐标转换
        |
        v
稳定性和安全边界检查 ---- 不通过 ---> 安全停止
        |
        v
选择垂直/折线接近路径
        |
        v
MoveIt 规划并低速执行
        |
        v
夹爪闭合与接触验证 ---- 未检测到接触 ---> 中止抬升
        |
        v
验证抬升 -> 放置 -> 释放 -> 返回中心观察位
```

## 硬件与软件

### 硬件

- Kinova Gen3，7 自由度
- Gen3 Vision Module（RGB-D）
- Robotiq 2F-85 夹爪（内部总线通信）
- Ubuntu 工作站与有线以太网

### 软件

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- `ros2_kortex`
- OpenCV、NumPy
- GStreamer
- Cyclone DDS

## 目录结构

```text
apple_grasp_safety_ws/
├── README.md
└── src/
    └── apple_grasp_scene/
        ├── launch/
        │   └── plan_only_robot.launch.py
        ├── scripts/
        │   ├── apple_locator.py
        │   ├── stable_apple_locator.py
        │   ├── vision_pick_apple.sh
        │   └── search_pick_place_apple.sh
        ├── src/
        │   ├── scene_plan_only.cpp
        │   └── gripper_check.cpp
        ├── CMakeLists.txt
        └── package.xml
```

核心文件：

- `apple_locator.py`：RGB-D 采集、颜色分割、点云生成、球面拟合和坐标转换
- `stable_apple_locator.py`：三帧检测、稳定性检查和边缘检查
- `scene_plan_only.cpp`：碰撞场景、路径规划、安全检查和受控轨迹执行
- `vision_pick_apple.sh`：抓取前校验、路径选择、夹爪控制与接触验证
- `search_pick_place_apple.sh`：多观察位搜索及完整抓取/放置任务编排

## 网络配置

项目当前使用 Gen3 默认地址：

```text
Robot IP: 192.168.1.10
Kortex TCP: 10000
Kortex UDP: 10001
```

主机有线网卡应设置为同一网段，例如：

```text
192.168.1.11/24
```

运行前检查：

```bash
ping 192.168.1.10
```

## 构建

该仓库只包含任务层代码。使用前需要安装并构建兼容 ROS 2 Humble 的 Kinova Kortex、MoveIt 2 和 Robotiq 驱动。

```bash
cd ~/apple_grasp_safety_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

本机开发环境中的 Kortex 驱动工作区路径为：

```text
/home/song/workspace/ros2_kortex_ws
```

在其他电脑上使用时，请替换脚本中的工作区路径，或将其改为环境变量。

## 运行

### 1. 启动真机驱动与 MoveIt

```bash
source /opt/ros/humble/setup.bash
source /home/song/workspace/ros2_kortex_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=false \
  use_internal_bus_gripper_comm:=true \
  launch_rviz:=false
```

### 2. 只执行视觉定位

该命令不会发送机械臂运动指令：

```bash
source /home/song/桌面/apple_grasp_safety_ws/install/setup.bash
ros2 run apple_grasp_scene stable_apple_locator.sh
```

成功时输出苹果中心、半径、腕部目标、点云数量和拟合误差等 JSON 数据。标注图像保存在：

```text
/tmp/apple_locator_annotated.jpg
```

### 3. 抓取路径预览

不传入 `--execute` 时只规划路径，不执行运动或夹爪命令：

```bash
ros2 run apple_grasp_scene vision_pick_apple.sh --path auto
```

### 4. 完整自动搜索与抓取

> 警告：下面的命令会真实移动机械臂和夹爪。执行前必须固定机械臂、清空工作区、确认急停可用，并由现场人员持续监护。

```bash
export APPLE_SEARCH_PICK_EXECUTE=YES
ros2 run apple_grasp_scene search_pick_place_apple.sh --execute
```

系统只有在环境变量和 `--execute` 参数同时存在时才允许真实运动。

## 当前安全约束

视觉抓取执行前会检查：

| 检查项 | 当前范围 |
|---|---:|
| 苹果 X | `0.55 ~ 0.80 m` |
| 苹果 Y | `-0.45 ~ -0.10 m` |
| 苹果中心高度 | `0.025 ~ 0.060 m` |
| 苹果半径 | `0.025 ~ 0.055 m` |
| 有效拟合点数 | `>= 1000` |
| 球面拟合中位残差 | `<= 0.003 m` |

此外，系统要求：

- 三帧 XY 位置变化不超过 `8 mm`
- 三帧半径变化不超过 `12 mm`
- 苹果轮廓不能接触图像边缘
- MoveIt 必须生成完整路径
- 夹爪必须检测到受阻接触，才允许执行验证抬升

这些参数只针对当前实机工作台完成过验证，不应直接用于其他机械臂安装方式或场景。

## 已知限制

- 视觉前端依赖红色 HSV 阈值，对光照和苹果颜色敏感
- 相机内外参与工具偏移针对当前设备标定
- 观察位和安全工作空间仍为硬编码配置
- 主要适用于固定桌面和单个苹果
- 尚未实现预抓取后的二次视觉闭环修正
- 尚未实现统一 ROS 2 Action 状态机和自动任务日志
- 不处理树枝、叶片、严重遮挡或动态障碍物

## 后续计划

- [ ] 将 Bash 任务编排重构为 ROS 2 Action 状态机
- [ ] 将观察位、安全边界和视觉阈值迁移到 YAML
- [ ] 增加预抓取位置的二次视觉定位与误差修正
- [ ] 增加失败重试、撤离和恢复策略
- [ ] 增加结构化任务日志与自动评测工具
- [ ] 增加实例分割后端，降低对红色阈值的依赖
- [ ] 补充仿真模式、自动测试和实验统计

## 安全说明

这是会控制真实机械臂运动的软件。任何真机运行都应满足：

1. 机械臂和工作台固定可靠。
2. 急停按钮可立即触达。
3. 工作空间内无人、无易损物和未知障碍物。
4. 首次修改参数后先使用规划模式验证。
5. 不得通过扩大边界或删除检查来绕过安全拒绝。

项目作者与贡献者不对未经现场验证的设备配置或运动参数负责。

