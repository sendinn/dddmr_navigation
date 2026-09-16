# DDDMR BEGINNER GUIDE

[العربية](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ar.md) | [繁體中文](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.zh-hant.md) | [简体中文](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.zh-hans.md) | [Русский](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ru.md) | [Deutsch](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.de.md) | [Español](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.es.md) | [한국어](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ko.md) | [Português](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.pt.md) | [Türkçe](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.tr.md) | [Tiếng Việt](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.vi.md) | [Français](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.fr.md) | [日本語](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/translations/README.ja.md)

This README is a beginner's guide to the DDDMR Navigation Stack. With both a gazebo quadruped robot example and a real robot guide, it's designed to help you get up and running fast, explore, and have fun along the way.

## Table of Contents

| # | Section | Description |
|:-:|:--------|:------------|
| 1 | [DDDMR Navigation with Gazebo](#-dddmr-navigation-with-gazebo) | Simulation demo with quadruped robot |
| 2 | [DDDMR Navigation with a Real Robot](#-start-dddmr-navigation-with-a-real-robot) | Deploy on your real robot (wheeled robot, quadruped, humanoid, and more) |

## 🖥️ Software Requirements
- **Ubuntu 22.04** (tested in 22.04, should support 24.04)
- **Docker**  [install Docker](https://docs.docker.com/engine/install/)
## ✨ DDDMR Navigation with Gazebo
This demo demonstrates how to run the DDDMR Navigation Stack in Gazebo with a quadruped robot.
- Build the required images to prepare the environment.
- Run the system in two terminals — one for the Gazebo world and another for the navigation stack. 

### 1. Create docker image
Clone the repo and run ./build.bash, please select **`x64_gz`**, which already contains all the necessary components for both navigation and Gazebo.
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Download navigation map
To play gazebo with dddmr_navigation, you will need to download demo navigation map (12.3MB).
```
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Prepare demo enviroment
#### Create a two docker container (gazebo and navigaiton)
> [!IMPORTANT] 
> The following command will start two interactive Docker containers using the image we built.  Please open **two separate terminals** to prepare the demo environment

#### 🖥️ Terminal 1  (create the container for gazebo system)

- ##### Step 1 (on host): create the container for Gazebo system
```
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_gazebo.bash
```
- ##### Step 2 (inside the gazebo container): build and launch
```
source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch go2_config gz_lidar_odom.launch.py
```

#### 🖥️ Terminal 2   (create the container for navigation system)

   - ##### Step 1 (on host): create the container for navigation system
```
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```
   - ##### Step 2 (inside the navigation container): build and launch
```
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Run demo 
- In the Gazebo demo, the map is already aligned, so you don’t need to set the initial pose unless you are mapping yourself
- Give the goal (3D Goal Pose) in RViz , then the robot will move to target point

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Known Issues
> [!WARNING]
> The following are currently observed behaviors. They are under investigation and will be fixed in future updates.
 - Gazebo: Occasional slipping on slopes 
 - Mapping: Duplicate floor layers  

#### 本地补充：Astrall/Odin1 的地面多层排查与修正

以下是本地分支的处理，不代表上游已解决所有重复地面问题。

在 Astrall 原始包与地图的离线比对中，发现部分保存为 ground 的高层点来自竖直表面，且在单个关键帧中已存在，不只是跨帧位姿漂移。

- 原有两点坡度判断不足以验证一个面是否为地面。新增 `imageProjection.ground_normal_check`，对地面候选和补片两端使用三维邻域协方差的最小特征向量检查法向，坡度限制沿用 `ground_slope_tolerance`。
- `ground_normal_radius: 0.20`、`ground_normal_min_neighbors: 6` 是本地初测参数。邻域稀疏、近似直线或非平面时不归类为地面；拒绝补片的点仍可保留为非地面观测，不能当成自由空间。
- 修正左右邻点跨行索引，跳过无效点，并要求左右两侧都有有效邻点。建图和导航的 Odin1 YAML 均开启法向检查；通用默认关闭。
- 单帧离线复现投影与法向判定：旧关键帧的 284 个高层点全部被拒绝；792 个低层点保留 578 个。该检查不是完整 ROS 管线或整包重建，不能用它宣称地面完整性、运行频率和导航安全已通过验收。
- 后续补充：法向邻域改用原始扫描回波，而不是稀疏投影格；插值补点也逐点检查邻域法向，且距最近原始回波不能超过 0.10 m，防止跨无观测空隙补出地面。

2026-09-16 整包离线验证（与上游说明分开）：使用 `astrall_2026_09_11_18_07_03`，0.5 倍速、外部里程计模式、关闭回环，生成 `maps/astrall_validation/2026_09_16_10_58_34`。保存 159 个关键帧、14,000 个聚合地面点，逐帧文件齐全，重建地面 XYZ 均有限。按 0.2 m XY 网格、跨帧重复支持及大于 0.12 m 高度间隙检查，轨迹 0.5 m 范围内未检出分离双层；轨迹到最近地面点的水平距离最大约 0.060 m。

全图仍有 3 个网格检出高低两组点。对疑似来源扫描的几何匹配表明，高层点有近水平原始回波支持，不能简单断定为重复地面，更不能为了让统计归零而压平真实物体/台阶。地面法向接近竖直，只证明局部表面近水平，不证明机械狗可以走上去。本次仅验证该包的地面分层改善与几何覆盖；日志仍出现里程计/点云时间差警告，尚未完成同步、实时吞吐、MCL、感知代价图和无运动规划验收，不能据此恢复自动行走。

已有 PCD 不会自动修复。需保留原地图，将原始包重建到新目录，并检查地面连续性、分层、定位与规划。不同高度的真实台阶/平台、同步或位姿漂移仍需分别分析，不应将所有地面直接压平。

#### 本地补充：扫描与外部里程计必须按时间配对

旧 `FeatureAssociation` 使用最近收到的里程计，处理点云落后时可能把不同时间的位姿套到扫描上；超过 1 秒后仅覆盖里程计时间戳并不能修复位姿。新的处理保存最多 2000 条里程计，优先查找扫描同时间戳的记录，否则在前后记录之间对位置线性插值、姿态四元数 SLERP。插值区间大于 0.25 秒、没有覆盖时间或 frame 不符时丢弃扫描，不外推。该阈值是当前实现的保守限制，不是雷达硬件规格。

里程计订阅线程只更新缓存；处理线程使用同一份匹配位姿完成特征处理、地图输入和 TF 输出，避免处理中途被新到的位姿覆盖。此改动影响共享该处理模块的外部里程计模式，不能自动修正旧地图；升级后须重新启动节点并重新建图验证。Web 扫描缓存只负责等待 TF，不能代替这一底层同步。

同步修复后原速整包验证地图：`maps/astrall_validation/2026_09_16_11_29_33`。短段独立核验的 29 条 `odom → base_link` TF 与原始包同时间戳位姿一致（位置误差 0，四元数误差约 1.57e-16）。原速重建地图按上述规则未检出分离双层，但关键帧数量与半速不同，不能据此保证所有地面或导航安全；详细限制见该地图目录的 `VALIDATION.md`。



---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Real Robot Tutorial Below 🤖 ━━━━━━━━━━</b>
</p>

---


## ✨ Start DDDMR Navigation with a Real Robot

This guide will walk you through setting up **3D navigation** on your real robot — from mapping to autonomous navigation.

DDDMR brings 3D navigation to your wheeled robot, quadruped, humanoid, and more. Let's get your robot moving!

### 1. Create docker image

Clone the repo and run ./build.bash, please select **`x64`** or **`l4t`** depending on your platform.

```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Note:** Our test platform uses Jetson Orin Nano, so we select **`l4t`**.

### 2. Test Platform

To quickly match this tutorial, here is the hardware setup we use:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Component | Model |
|:----------|:------|
| Robot | Quadruped (Lite3) |
| LiDAR | Dome LiDAR (RoboSense Airy), tilted 45° |
| Computer | Jetson Orin Nano |
| Power | External battery for LiDAR & Jetson |
| Connection | Ethernet (robot, Jetson, and LiDAR communicate over network) |

> **Note:** You don't need the exact same hardware. As long as your system meets the requirements below, DDDMR will work.

#### Requirements

| Item | Topic | Message Type | Description |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command to control your robot |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D LiDAR point cloud (multi-layer, dome, solid, etc.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | Odometry with TF (error < 10% will be better) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF tree: `odom` → `base_link` → `lidar_link` |

> **Note:** 
> - DDDMR can be used on **wheeled robots, quadrupeds, and humanoids** as long as they meet the requirements above.
> - `/cmd_vel` is the **input** to your robot for motion control.
> - `/lidar_point_cloud`, `/odom`, `/tf` are **outputs** from your robot, used by DDDMR for mapping and localization.

#### TF Tree Structure

Make sure your `frame_id` in LiDAR topic matches your TF tree:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### TF Configuration Example

Here is the TF configuration using our quadruped robot as an example. The LiDAR is mounted at `x=0.2m`, `z=0.22m` from `base_link`, and pitched down by 45°:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Note:** 45° ≈ 0.785 rad. Pitch down.

#### 👉 Advanced (Optional)

| Feature | Description |
|:--------|:------------|
| [3D Odometry](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Better localization & mapping on uneven terrain |

---

### 3. 🚀 RUN Mapping + Navigation for robot

DDDMR supports three navigation workflows:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Note:** This tutorial follows **② Online mapping + Nav** workflow.

---


### 3.1 🗺️ Mapping

Before starting, make sure your robot is publishing the required topics:

- Odometry topic
- LiDAR point cloud topic  
- TF (`odom` → `base_link` → `lidar_link`)

Then launch the mapping:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

After launching, you should see the RViz interface like this:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

When you drive your robot around, you will notice:
- **Key frames** — should increase as the robot moves
- **Feature points** — extracted features from the environment
- **Ground points** — detected ground surface
- **2D projection** — helps you understand the scene and LiDAR FOV

#### Save the Map

After you finish mapping the area, open a new terminal and run:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

When you see `Create dir:` in the terminal, it means the map has been saved. If you are done mapping, you can close the mapping node.

The map will be saved in `/tmp`. You can move the folder to `/root/dddmr_bags`:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Note:** The folder name `2026_03_28_19_25_15` is just an example. Your folder name will be different based on when you run the mapping.

---

### 3.2 📍 Localization + Navigation

First, open the configuration file and set your map path:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

Change `pose_graph_dir` to your map folder:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Note:** Replace `2026_03_28_19_25_15` with your actual map folder name.

Then launch the localization and navigation:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Step 1: Give Initial Pose

Click **「3D Pose Estimate」** in RViz toolbar and select a ground point that matches your robot's real-world position.

If the initial pose is correct, you will see the observed point cloud overlap well with the map features.

##### Step 2: Send Goal

Once initialization is done, click **「3D Goal Pose」** and select a ground point as the goal.

The robot should start moving toward the goal.
