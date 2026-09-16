# 🤖 dddmr_navigation

<!-- 上游 README 中文翻译开始 -->

> [!NOTE]
> 本节是上游 `dfl-rlab/dddmr_navigation` README 的中文翻译，保留原有结构、链接和含义。翻译结束后的“本地补充”是针对本机 Astrall 机械狗与 Odin1 雷达编写的使用说明，不属于上游原文。

## 🧪 自动化 CI/CD 与多雷达测试套件

对于初学者来说，配置具有非零倾斜角（俯仰角、横滚角、偏航角）的三维激光雷达通常非常困难，并且经常导致地图损坏、地面滤波错误或坐标变换异常。

最新版本引入了自动化 **CI/CD 测试套件**，可在部署前验证传感器配置和导航兼容性：

- **角度与俯仰验证：** 自动测试水平安装、向前倾斜安装以及自定义角度安装的雷达几何配置。
- **TF 与点云精度：** 确保不同安装角度下的空间变换、地面投影和障碍物清除保持一致。
- **适合初学者的基准测试：** 预先配置的测试场景可作为自定义机器人可靠的参考配置。

### 🔗 快速链接

- **在本地运行测试：** 详细步骤参见 [CI/CD 文档](https://github.com/dfl-rlab/dddmr_navigation/tree/main/CICD_setup#dddmr-lego-loam-ci)。

<table align='center'>
  <tr width="100%">
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/CICD_setup/airy_t45_2x.gif" width="256" height="157"/><p align='center'>Airy 倾斜 45 度</p></td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/CICD_setup/c16_t0_2x.gif" width="256" height="157"/><p align='center'>C16 不倾斜安装</p></td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/CICD_setup/mid360_t180_2x.gif" width="256" height="157"/><p align='center'>Mid360 横滚 180 度</p></td>
  </tr>
</table>

## 🚀 Go2 仿真器

我们已经将使用 Unitree Go2 的 Gazebo 模型集成到 DDDMR 导航栈中，从而支持真正的三维导航仿真与测试。通过将新型 Go2 四足机器人与 DDDMR 导航栈结合，可以探索远超传统二维导航框架的能力。

现在即可开始仿真，体验仅使用 Nav2 难以实现的多楼层建图、坡道导航以及复杂环境障碍物处理。

[👾 使用 DDDMR Navigation 体验 Go2](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_beginner_guide)

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_gz.gif" width="700" height="420"/>
</p>

---

> [!NOTE]
> DDDMR Navigation Stack 用于处理 [Nav2](https://github.com/ros-navigation/navigation2) 难以应对的问题，例如多层地面的建图与定位、立体结构中的路径规划，以及在三维点云地图中标记和清除感知障碍物。

<table align='center'>
  <tr width="100%">
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/multilevel_map.gif" width="400" height="260"/><p align='center'>多楼层地图</p></td>
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/obstacle_avoidance.gif" width="400" height="260"/><p align='center'>坡道障碍物避让</p></td>
  </tr>
  <tr width="100%">
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/mapping_navigating.gif" width="400" height="260"/><p align='center'>边建图边导航</p></td>
    <td width="40%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_semantic_segmentation/dddmr_semantic_segmentation_to_pointcloud.gif" width="400" height="260"/><p align='center'>语义分割与导航（敬请期待🔥）</p></td>
  </tr>
</table>

DDDMR Navigation（3D Mobile Robot Navigation，三维移动机器人导航）是一套支持用户在三维环境中完成建图、定位和自主导航的导航栈。

下图展示了二维导航栈与 DDD（三维）导航的对比。DDDMR 为移动平台在三维环境中导航提供了一套完整方案，选择 DDD 导航具有以下优势：

✨ DDD 移动机器人和二维移动机器人的标准工作流程相同，因此二维导航栈用户可以较容易地迁移到 DDD 导航：

1. 使用项目提供的软件包和工具建图并修整地图。
2. 关闭建图，通过给定初始位姿，使用 MCL 对机器人进行定位。
3. 向机器人发送目标点，机器人使用全局规划器计算路径，并使用局部规划器避障。

✨ DDD 导航不再受地形限制，例如可以处理工厂坡道或无障碍通道。

✨ DDD 导航已在多个领域经过充分测试，并基于低成本硬件运行，例如 16 线雷达、Intel NUC/Jetson Orin Nano 和消费级 IMU。项目致力于尽可能降低整套方案的成本。

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/navigation_diagram.png" width="780" height="560"/>
</p>

## 引用

如果使用了本项目的全部或部分代码，请引用：

```bibtex
@software{dddmr_navigation_dfl-rlab,
  author = {CM, PS, Tarek Taha},
  title = {dddmr_navigation: 3D Mobile Robot Navigation},
  url = {https://github.com/dfl-rlab/dddmr_navigation},
  year = {2025}
}
```

😵‍💫 已经有机器人，但不知道从哪里开始？[点击查看初学者指南](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_beginner_guide/README.md) 😵‍💫

## 🏁 各软件包的详细文档

<details><summary><b>💡 点击查看建图</b></summary>

https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_lego_loam

</details>
<details><summary><b>💡 点击查看定位</b></summary>

https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_mcl_3dl

</details>
<details><summary><b>💡 点击查看感知</b></summary>

https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_perception_3d

</details>
<details><summary><b>💡 点击查看全局规划器</b></summary>

https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_global_planner

</details>
<details><summary><b>💡 点击查看局部规划器</b></summary>

https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_local_planner

</details>
<details><summary><b>💡 点击查看 Move Base</b></summary>

https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_p2p_move_base

</details>

## DDD 导航功能演示

<table align='center'>
  <tr width="100%">
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/mapping.gif" width="400" height="260"/><p align='center'>三维建图</p></td>
    <td width="50%"><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/global_planner.gif" width="400" height="260"/><p align='center'>三维全局规划</p></td>
  </tr>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/local_planner.gif" width="400" height="260"/><p align='center'>三维局部规划</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_navigation/navigation.gif" width="400" height="260"/><p align='center'>三维导航</p></td>
  </tr>
  <tr width="100%">
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/scanning_lidar_demo.gif" width="400" height="260"/><p align='center'>支持不同传感器（Unitree G4）</p></td>
    <td><img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/perception_3d/multi_depth_camera_demo.gif" width="400" height="260"/><p align='center'>支持不同传感器（深度相机）</p></td>
  </tr>
</table>

<!-- 上游 README 中文翻译结束 -->

---

<!-- 本地补充开始 -->

# 本地补充：DDDMR 建图输入与配置指南（非上游原文）

> [!IMPORTANT]
> 从这里开始是本仓库针对 **Astrall 机械狗、Odin1 雷达和当前 `sendinn` 分支源码** 增加的本地说明，不是上游 README 的翻译。参数和文件行为以本仓库当前源码为准；雷达视场、安装外参和话题名必须再用实机数据核对。

## 1. 建图数据链

```text
雷达 PointCloud2 ──remap──> lslidar_point_cloud ──> 投影/地面分割/特征提取
                                                       │
里程计 Odometry ──remap──> odom ───────────────────────┤
                                                       ▼
                                      激光里程计、关键帧、位姿图、地图文件

TF: base_ground_frame <──> 点云 header.frame_id
```

建图节点内部使用固定名称 `lslidar_point_cloud` 和 `odom`。实际传感器话题应在 launch 文件中通过 remap 接入，不建议为了适配设备直接修改算法源码。

## 2. 建图需要的话题与 TF

| 输入 | ROS 类型 | 是否必需 | 用途与要求 |
|---|---|---:|---|
| 雷达点云 | `sensor_msgs/msg/PointCloud2` | 是 | 输入点必须具有有效 `x/y/z`；`header.stamp`、`header.frame_id` 必须正确。实际话题 remap 到 `lslidar_point_cloud`。 |
| 外部里程计 | `nav_msgs/msg/Odometry` | 取决于配置 | `laser.odom_type: wheel_odometry` 时必需，实际话题 remap 到 `odom`。时间戳应与点云使用同一时钟源。 |
| TF 静态外参 | `/tf_static` | 是 | 必须能够查询 `laser.base_ground_frame` 与点云坐标系之间的变换，包含雷达安装平移和 roll/pitch/yaw。 |
| TF 动态变换 | `/tf` | 取决于整机系统 | 机器人状态发布、可视化和后续导航通常需要。不要同时让多个节点发布相互冲突的同名 TF。 |
| 仿真时钟 | `/clock` | 仅仿真 | 只有节点启用 `use_sim_time: true` 时需要；实机一般使用系统时间。 |

当前建图入口没有要求单独订阅 IMU 话题。若需要 IMU，可先在外部融合为里程计，再将结果 remap 到 `odom`。

建图前可快速核对：

```bash
ros2 topic list
ros2 topic info /实际点云话题 --verbose
ros2 topic echo /实际点云话题 --once --field header
ros2 topic hz /实际点云话题
ros2 topic echo /实际里程计话题 --once --field header
ros2 run tf2_ros tf2_echo base_link 雷达frame_id
```

### 2.1 ROS 2 话题重映射（remap）

DDDMR 源码使用固定的内部话题名，而不同机器人驱动发布的话题名通常不同。重映射是在**不修改算法源码**的情况下，把节点内部话题连接到实际机器人话题。

映射方向为：

```text
DDDMR 节点内部话题  ──remap──>  机器人实际话题
lslidar_point_cloud  ──────────>  /astrall/lidar/points
odom                 ──────────>  /astrall/base/odometry
cmd_vel              ──────────>  /cmd_vel
```

当前 Astrall bringup 在 Python launch 中使用以下写法：

```python
Node(
    package="lego_loam_bor",
    executable="lego_loam",
    parameters=[config_file],
    remappings=[
        ("lslidar_point_cloud", lidar_topic),  # DDDMR 内部点云 -> 实际雷达点云
        ("odom", odom_topic),                  # DDDMR 内部里程计 -> 实际机身里程计
    ],
)
```

括号左侧是源码中创建订阅者/发布者时使用的名称，右侧是机器人实际使用的名称。不要写反。例如：

```python
("lslidar_point_cloud", "/astrall/lidar/points")
```

表示 DDDMR 原本订阅 `lslidar_point_cloud`，运行后改为订阅 `/astrall/lidar/points`；它不是把 `/astrall/lidar/points` 再发布成一个新话题。

本机建图 launch 的默认参数为：

| launch 参数 | 默认实际话题 | 连接到 DDDMR 内部话题 | 数据方向 |
|---|---|---|---|
| `lidar_topic` | `/astrall/lidar/points` | `lslidar_point_cloud` | Astrall → DDDMR |
| `odom_topic` | `/astrall/base/odometry` | `odom` | Astrall → DDDMR |

本机导航 launch 还包含：

| launch 参数 | 默认实际话题 | 连接到 DDDMR 内部话题 | 数据方向 |
|---|---|---|---|
| `lidar_topic` | `/astrall/lidar/points` | `lslidar_point_cloud` | Astrall → 特征提取/MCL |
| `odom_topic` | `/astrall/base/odometry` | `odom` | Astrall → MCL/Move Base |
| `cmd_vel_topic` | `/cmd_vel` | `cmd_vel` | Move Base → Astrall 控制节点 |

如果实际驱动换了话题名，优先通过 launch 参数覆盖：

```bash
# 使用其他点云和里程计话题建图
ros2 launch astrall_dddmr_bringup mapping.launch.py \
  lidar_topic:=/my_lidar/points \
  odom_topic:=/my_robot/odometry

# 驱动已由其他终端启动时，不重复启动 Astrall 驱动
ros2 launch astrall_dddmr_bringup mapping.launch.py \
  start_driver:=False \
  lidar_topic:=/my_lidar/points \
  odom_topic:=/my_robot/odometry
```

直接运行 DDDMR 可执行文件时，也可以使用 ROS 2 命令行重映射：

```bash
ros2 run lego_loam_bor lego_loam --ros-args \
  --remap lslidar_point_cloud:=/my_lidar/points \
  --remap odom:=/my_robot/odometry \
  --params-file /绝对路径/mapping.yaml
```

重映射后可检查实际连接关系：

```bash
ros2 node list
ros2 node info /实际节点名
ros2 topic info /astrall/lidar/points --verbose
ros2 topic info /astrall/base/odometry --verbose
```

> [!WARNING]
> Remap 只改变 ROS 话题名称，不会改变消息类型、QoS、点云坐标、`header.frame_id`、时间戳或 TF。即使话题已经连通，点云仍必须是 `sensor_msgs/msg/PointCloud2`，里程计仍必须是 `nav_msgs/msg/Odometry`，并且 `base_link`、`lidar_link`、`odom` 之间的 TF 必须正确。

> [!CAUTION]
> 导航中的 `cmd_vel` 是控制输出。修改 `cmd_vel_topic` 只改变输出目标，不代表已经验证定位或取得机械狗控制权。实机运动前必须确认 MCL 已收敛、规划路径有效、障碍感知正常，并显式启用控制。

## 3. YAML 基本结构

下面只列出当前源码会读取的主要参数。它是结构示例，不是可直接套用所有雷达的标定结果。

```yaml
# 点云投影、地面分割节点
lego_loam_ip:                              # 节点名；必须与 launch 中启动的 ImageProjection 节点名一致。
  ros__parameters:                        # ROS 2 参数入口，不能省略。
    use_sim_time: false                   # 是否使用 /clock；实机为 false，rosbag/仿真通常为 true。

    laser:                                # 雷达扫描模型和运动估计来源。
      num_vertical_scans: 16              # 深度图垂直行数；应与雷达扫描组织匹配。
      num_horizontal_scans: 1000          # 完整 360° 的逻辑列数；前向雷达未观测列保持为空。
      horizontal_fov: 360.0               # Astrall 本地扩展字段：雷达实际水平视场（度）；用于按可见区域校验最低输入点数，默认 360 保持上游旋转雷达行为。
      scan_period: 0.1                    # 单帧/单圈周期，单位秒；0.1 表示约 10 Hz。
      vertical_angle_bottom: -15.0        # 垂直视场下边界，单位度。
      vertical_angle_top: 15.0            # 垂直视场上边界，单位度。
      odom_type: wheel_odometry           # wheel_odometry 使用输入 odom；laser_odometry 使用激光里程计。
      base_ground_frame: base_link        # 地面判断参考坐标系；必须能与点云 frame 建立 TF。

    imageProjection:                      # 深度图投影、点云分割和地面提取参数。
      segment_theta: 60.0                 # 相邻点分割角阈值，单位度；改变物体聚类结果。
      segment_valid_point_num: 5          # 有效分割簇的最少点数；增大可滤噪但可能漏掉小物体。
      segment_valid_line_num: 3           # 有效簇至少覆盖的扫描线数；低线束雷达不宜设得过大。
      minimum_detection_range: 0.5        # 最近有效距离，单位米；近于该值的点被忽略。
      maximum_detection_range: 100.0      # 最远有效距离，单位米；远于该值的点不参与投影。
      distance_for_patch_between_rings: 0.2 # 相邻扫描环地面补片的最大间距，单位米。
      stitcher_num: 1                     # 拼接帧数；增大可加密非重复扫描，但运动时容易产生拖影。

      ground_fov_bottom: -0.785398        # 地面候选垂直视场下边界，单位弧度（约 -45°）。
      ground_fov_top: 0.0                 # 地面候选垂直视场上边界，单位弧度。
      ground_positive_start: 0.0          # 正方位地面检测扇区起点，单位弧度。
      ground_positive_stop: 1.0472        # 正方位地面检测扇区终点，单位弧度（约 +60°）。
      ground_negative_start: 0.0          # 负方位地面检测扇区起点，单位弧度。
      ground_negative_stop: -1.0472       # 负方位地面检测扇区终点，单位弧度（约 -60°）。
      ground_slope_tolerance: 0.35        # 相邻地面片允许的坡度差，单位弧度（约 20°）。
      ground_dz_tolerance: 0.15           # 相邻候选地面点允许的高度差，单位米。
      use_sensor_height_to_filter_out_ground: false # 是否结合传感器高度过滤地面；要求高度和外参准确。
      patch_first_ring_to_baselink: false # 是否把首圈地面补到 base_link；前置雷达误补风险较高。

# 特征提取与激光里程计节点
lego_loam_fa:                              # 节点名；必须与 launch 中的 FeatureAssociation 节点名一致。
  ros__parameters:                        # ROS 2 参数入口。
    use_sim_time: false                   # 时钟来源必须与点云、里程计节点保持一致。

    featureAssociation:                   # 边缘/平面特征提取和关联参数。
      edge_threshold: 0.1                 # 边缘曲率阈值；影响边缘特征的数量与强度。
      surf_threshold: 0.1                 # 平面曲率阈值；影响平面特征的数量与质量。
      nearest_feature_search_distance: 3.0 # 特征最近邻搜索距离，单位米；过大易误匹配且更耗时。

    mapping:                              # 特征节点到地图优化节点的数据开关。
      to_map_optimization: true           # 是否向地图优化发布特征；false 时不会继续累计关键帧地图。

# 关键帧、位姿图与地图优化节点
lego_loam_mo:                              # 节点名；必须与 launch 中的 MapOptimization 节点名一致。
  ros__parameters:                        # ROS 2 参数入口。
    use_sim_time: false                   # 应与 IP、FA 及传感器使用相同的时钟模式。

    mapping:                              # 关键帧、局部地图、回环与 TF 参数。
      use_external_odometry_only: false   # true：直接提交外部里程计增量并跳过 scan-to-map；回环仍可独立启用。
      enable_loop_closure: true           # 是否启用 ICP 回环检测；错误回环可能拉坏整张地图。
      history_keyframe_search_radius: 15.0 # 回环候选的空间搜索半径，单位米。
      history_keyframe_search_num: 25     # 回环匹配使用的历史关键帧数量；越大计算量越高。
      history_keyframe_fitness_score: 0.3 # 接受回环的 ICP 拟合分数上限；越小越严格。
      surrounding_keyframe_search_num: 50 # 构建局部匹配地图时使用的周围关键帧数量。
      angle_between_key_frame: 0.2        # 新建关键帧的最小旋转变化，单位弧度（约 11.5°）。
      distance_between_key_frame: 0.5     # 新建关键帧的最小平移变化，单位米。
      ground_edge_threshold_num: 50       # 地面边缘判定所需的点数阈值；过大可能漏掉稀疏边缘。
      broadcast_external_odom_tf: false   # 是否广播外部里程计相关 TF；须避免与现有 TF 发布者冲突。
      generate_testing_pg: false          # 是否生成测试用位姿图数据；正常实机建图通常关闭。
```

> [!WARNING]
> 单位并不统一：`vertical_angle_bottom/top` 和 `segment_theta` 使用**度**；`ground_fov_*`、`ground_positive_*`、`ground_negative_*`、`ground_slope_tolerance`、`angle_between_key_frame` 使用**弧度**。填错单位会直接破坏地面分割和地图几何。

启动时应确认 YAML 确实加载到了目标节点。仅修改文件但 launch 仍引用其他 YAML，不会产生任何效果。

## 4. 关键参数分别影响什么

### 4.1 雷达扫描模型

| 参数 | 含义 | 配置错误的表现 |
|---|---|---|
| `num_vertical_scans` | 投影深度图的垂直行数 | 点被投到错误扫描线，地面呈多层、特征断裂或大量点丢失。 |
| `num_horizontal_scans` | 一整圈 `2π` 的逻辑水平列数 | 水平角分辨率错误，点云被拉伸、重复或稀疏。前向扇形雷达仍使用完整圆周的逻辑列数，未观测列保持空白。 |
| `horizontal_fov` | 雷达实际可见的水平视场角（度） | 只修正最低有效点数校验的可见区域比例，不改变完整 `2π` 投影和水平角分辨率；未配置时默认为 `360.0`。 |
| `scan_period` | 一帧/一圈的周期（秒） | 影响运动相关处理；与真实频率不符时，移动中更容易出现重影和轨迹旁障碍带。 |
| `vertical_angle_bottom/top` | 雷达有效垂直视场下、上边界（度） | 地面投影错行，地面可能被当作障碍，墙面也可能被误判为地面。 |
| `odom_type` | 使用外部轮式/融合里程计，或激光里程计 | 选择 `wheel_odometry` 却没有稳定 `odom` 时，地图会漂移或停更。 |
| `base_ground_frame` | 地面分割参考坐标系 | 外参不对会使地面倾斜、机器人自身点残留，并影响保存的关键帧姿态。 |

`num_horizontal_scans` 不是“实际只看见多少度”。例如水平分辨率为 `0.5°` 时，完整圆周逻辑列数通常是 `360 / 0.5 = 720`；Odin1 只有前方扇形视场时，后方未观测列应为空，不能把扇形强行拉满 360°，也不能把未观测区当成已清空区域。

`horizontal_fov` 是 Astrall 分支为扇形雷达新增的兼容字段。原上游有效性检查实际要求点数不低于 `num_vertical_scans * num_horizontal_scans * 0.1 * stitcher_num`，只是旧错误日志把完整的 `num_vertical_scans * num_horizontal_scans` 打印成了 `Expecting`。新增字段按 `horizontal_fov / 360` 缩放这个下限；它不改变投影坐标计算。

### 4.2 投影、地面与点云过滤

| 参数 | 主要作用 | 调大/调宽后的典型影响 |
|---|---|---|
| `minimum_detection_range` | 删除过近回波 | 可排除机身和近场噪声，但过大会失去贴近机器人的障碍物。 |
| `maximum_detection_range` | 删除过远回波 | 可抑制远处稀疏噪声，但会缩短地图和定位的可见范围。 |
| `segment_theta` | 控制相邻点聚类/分割判据 | 会改变小物体、墙面和离群点的分组结果；应同时观察 `segmented_cloud` 与 `outlier_cloud` 验证。 |
| `segment_valid_point_num` | 有效分割簇所需点数 | 越大越能滤除小噪声，也越容易漏掉细杆等小障碍。 |
| `segment_valid_line_num` | 有效簇需覆盖的扫描线数 | 越大越严格，低线束或扇形雷达可能漏检。 |
| `stitcher_num` | 非重复扫描的帧拼接数量 | 增大可补足稀疏扫描，但移动时更易重影；旋转式雷达一般从 1 开始。 |
| `ground_fov_bottom/top` | 参与地面检测的垂直角范围 | 过宽会把墙、台阶或机身纳入地面候选；过窄会让地面残留为障碍。 |
| `ground_positive_start/stop` | 正方向水平扇区的地面检测范围 | 扩大后处理更多该方向点；范围或符号错误会处理错误象限。 |
| `ground_negative_start/stop` | 负方向水平扇区的地面检测范围 | 与正方向共同覆盖雷达可见地面；不要用它虚构雷达未观测的后方区域。 |
| `ground_slope_tolerance` | 允许相邻地面片的坡度差 | 过大会把坡边/低障碍当地面，过小会把坡道或颠簸地面当障碍。 |
| `ground_dz_tolerance` | 允许地面片高度变化 | 过大可能吞掉路沿，过小会使地面产生多层红色膨胀点。 |
| `use_sensor_height_to_filter_out_ground` | 是否结合传感器高度过滤地面 | 需要准确的雷达高度和外参；不准确时可能大量误删或误留。 |
| `patch_first_ring_to_baselink` | 是否将首圈地面补到基座附近 | 可填补近场地面空洞，但配置不当会在机身附近生成错误地面。 |

`ground_positive_stop` 只影响正方向扇区中参与地面检测的水平角终点。它不会改变雷达原始点云，也不会自动扩大真实视场；设置过小会使扇区外地面进入非地面/障碍处理，设置过大则可能让不该参与的点进入地面判断。

### 4.3 特征、关键帧与回环

| 参数 | 主要作用 | 风险 |
|---|---|---|
| `edge_threshold` | 边缘特征阈值 | 特征过少会降低匹配约束，过多会增加噪声和计算量。 |
| `surf_threshold` | 平面特征阈值 | 影响地面、墙面等平面特征数量和配准稳定性。 |
| `nearest_feature_search_distance` | 最近特征搜索范围 | 太小匹配不足，太大可能产生错误对应且计算更慢。 |
| `distance_between_key_frame` | 平移多少米后生成新关键帧 | 太小导致地图庞大、重复点多；太大导致地图稀疏、转弯处信息不足。 |
| `angle_between_key_frame` | 旋转多少弧度后生成新关键帧 | 太小关键帧过密，太大可能漏掉原地转向过程。 |
| `use_external_odometry_only` | 是否直接采用外部里程计并跳过 DDDMR scan-to-map | `true` 依赖外部里程计精度；`false` 会用当前帧特征对局部地图做二次优化。它不控制独立的 ICP 回环。 |
| `enable_loop_closure` | 是否启用回环闭合 | 关闭后长距离漂移不会被校正；错误回环则会拉坏整张地图。 |
| `history_keyframe_*` | 回环候选范围、数量与拟合阈值 | 过严无法闭环，过松可能误闭环。修改后必须检查轨迹和点云是否突跳。 |

## 5. 不同雷达如何配置

先从厂家驱动或实测数据确认以下信息，再填写 YAML：

1. 点云是否为单帧完整扫描，是否是非重复扫描模式。
2. 水平/垂直真实视场、角分辨率、扫描线或投影行数。
3. 实际发布频率与 `scan_period`。
4. 点云坐标轴方向和 `header.frame_id`。
5. 雷达相对 `base_link` 的精确平移、roll、pitch、yaw。

| 雷达类型 | 推荐起点 | 特别注意 |
|---|---|---|
| 传统 16 线旋转雷达，如 C16/XT16 | 参考 `mapping_c16_t0.yaml`，按真实线数和垂直角修改 | 通常一帧覆盖 360°，`stitcher_num` 从 1 开始；扫描周期按实际转速填写。 |
| 固态/大视场雷达，如 Airy、JT128 | 参考仓库对应测试配置 | 投影行列不能只凭“线数”猜测；安装倾角必须通过 TF 表达。 |
| 非重复扫描雷达，如部分 Mid360 模式 | 参考 `mapping_mid360_*.yaml`，`stitcher_num` 可从 2 开始验证 | 拼接越多，静止时越密，但运动重影越明显。 |
| Odin1 前向扇形雷达 | 使用本机 `odin1_mapping.yaml` 作为测试起点，再根据真实 PointCloud2 校正 | 只处理实际可见扇区。后方没有回波代表“未知”，不代表“无障碍”。机械狗运动和俯仰会放大拼帧与外参误差。 |

本机 Odin1 测试模板位于：

```text
/home/nvidia/sed_ros2/sed_astrall/sw01/ros/astrall_dddmr_bringup/config/odin1_mapping.yaml
```

模板当前采用的关键假设包括：180 个垂直投影行、720 个完整圆周逻辑列（`0.5°/列`）、垂直视场约 `-45°～45°`、扫描周期 `0.1 s`、前方地面水平扇区约 `±60°`。这些只是本机测试起点，不等同于 Odin1 所有固件或工作模式的官方参数。必须通过实机点云验证，尤其不要仅凭型号名称复制参数。

仓库中的参考配置位于：

```text
src/dddmr_lego_loam/lego_loam_bor/test/config/
├── mapping_airy_t45.yaml
├── mapping_c16_t0.yaml
├── mapping_jt128_t45.yaml
├── mapping_mid360_t13.yaml
└── mapping_mid360_t180.yaml
```

## 6. 建图过程中会发布哪些点云

以下是当前建图程序的主要中间输出；实际完整话题名可能受到 namespace 和 remap 影响：

| 输出话题 | 内容 | 用途 |
|---|---|---|
| `full_cloud_info` | 投影后的完整点云及附加信息 | 算法内部处理与调试投影是否正确；当前对外发布调用被注释。 |
| `ground_cloud` | 被识别为地面的点 | 检查地面是否单层、坡道是否连续，以及机身/墙面是否误入。 |
| `segmented_cloud` | 分割后的有效点 | 后续特征提取和里程计匹配的主要输入。 |
| `segmented_cloud_pure` | 当前 `sendinn` 源码在地面提取阶段保留的非地面/障碍候选点 | 作为导航阶段动态障碍感知输入；当前版本并不等同于“所有有效连通分割簇”，详见 7.1 节。 |
| `outlier_cloud` | 被判为离群的点 | 诊断噪声、稀疏回波以及分割阈值；当前投影节点对外发布调用被注释。 |
| `laser_cloud_sharp` | 强边缘特征 | 用于姿态与结构约束。 |
| `laser_cloud_less_sharp` | 较宽松的边缘特征 | 用于建图匹配。 |
| `laser_cloud_flat` | 强平面特征 | 用于平面约束。 |
| `laser_cloud_less_flat` | 较宽松的平面特征 | 用于建图和地图表达。 |
| `laser_odom_to_init` | 激光里程计结果 | 表示建图过程中的相对运动估计，不等同于导航阶段最终 MCL 定位。 |

地面点正常情况下可以有坡度和厚度，但同一处静止地面不应长期出现明显的多层平行结构。多层地面常见原因是时间戳不同步、动态 TF/外参错误、扫描周期不对、拼帧过多或机械狗运动时机身俯仰未被正确补偿。

### 6.1 本地补充：地面点、边缘点、平面点的生成步骤与先后顺序

本节依据当前本地源码说明，不是上游原文。主要实现位于 `src/dddmr_lego_loam/lego_loam_bor/src/imageProjection.cpp`、`featureAssociation.cpp` 和 `include/mapOptimization.h`；下述数值表示当前实现/配置示例，不代表所有雷达的固定要求。

整体顺序如下。地面提取有独立输出分支，并不是所有地面点都必须再经过边缘/平面特征筛选：

```text
原始一帧 XYZ 点云
  → 去除无效点、安装角处理、按水平角/垂直角投影成距离图
  → 地面判断 ──→ 经验证的地面补点、降采样 → patched_ground
  → 距离图连通分割，整理 segmented_cloud 和地面标记
  → 特征处理前的坐标/扫描时间处理
  → 计算距离平滑度
  → 标记遮挡边界和不可靠点
  → 每条扫描行分成 6 段：先选边缘，再选严格平面，再收集宽松表面
  → 结合位姿处理、关键帧筛选，将各类点保存并拼接为地图
```

#### 第一步：把真实回波点放入距离图

雷达测量物体反射回来的信号，驱动输出三维点 `(x, y, z)`。这里的邻域点来自真实点云，不是程序为了计算法向而凭空生成的。

对每个有效点计算：

```text
距离 r = sqrt(x² + y² + z²)
水平角 α = atan2(y, x)
垂直角 β = asin(z / r)
```

用水平角确定列、垂直角确定行，把距离及对应 XYZ 写入投影网格。`num_vertical_scans` 是投影行数，`num_horizontal_scans` 是投影列数，不是这一帧的点数。当前水平投影按完整 360° 划分；720 列对应每列 0.5°，Odin1 只有前方约 120° 有观测，并不会因此拥有后方回波。没有回波的格子保持无效。

#### 第二步：识别地面，并生成独立的地面点云

`zPitchRollFeatureRemoval()` 同时处理“供特征提取使用的地面标记”和“用于保存地图的补点地面”，二者条件不同，不能认为是完全相同的点集。

**A. 地面标记：** 比较同一列、相邻垂直行的两个有效点，计算连线倾角：

```text
Δx = x上 - x下，Δy = y上 - y下，Δz = z上 - z下
θ = atan2(Δz, sqrt(Δx² + Δy²))
```

当前该分支要求 `|θ| ≤ 5°`，并在启用 `ground_normal_check` 时要求两个端点附近的三维法向检查都通过，然后设置地面标记。此处连线使用原始投影坐标；后面的补点坡度判断使用安装俯仰角修正后的坐标，不能把两个阈值混为一谈。

**B. 邻域法向检查：** 在安装俯仰角修正后的原始点云中搜索邻居，当前 Odin1 配置搜索半径为 0.2 m，至少需要 6 个点。计算邻域均值及协方差累加矩阵，取最小特征值对应的特征向量作为法向；排除近似直线、明显不共面等不可靠邻域，再要求法向接近竖直方向：

```text
|法向的 z 分量| ≥ cos(ground_slope_tolerance)
```

例如容差为 0.3 rad 时，右侧约为 0.955。水平地面的法向接近 `(0, 0, 1)`，能够通过；竖直墙面的法向接近水平方向，不能通过。因此“两点高度相近”本身不足以证明是地面。当前检查还要求附近 0.1 m 内存在原始观测支持。

**C. 地面补点：** 在配置的地面角度范围内，检查两个端点法向、左右邻点是否存在以及高度是否连续。当前左右高度差限制为 0.05 m；再同时检查垂直相邻点的坡度和高度差：

```text
|θ| ≤ ground_slope_tolerance
|Δz| ≤ ground_dz_tolerance
```

当前示例分别是 0.3 rad 和 0.1 m，任何一项不满足都不能按该分支补地面。若启用传感器高度过滤，还需通过相应高度条件。两端距离小于 `distance_for_patch_between_rings` 才允许插值；插值间隔约为 0.1 m，每个候选补点也要通过观测支持和法向检查（启用法向检查时）。最后对 `patched_ground` 做 0.1 m 体素降采样。

例如同一地面上的 `A=(1.37, 0, -0.50)`、`B=(1.45, 0, -0.50)`，连线 `Δz=0`、倾角为 0°；若邻域检查也通过，可作为地面候选。竖直墙上两点即使只相差 0.04 m 高度，只要水平距离接近 0，连线倾角仍接近 90°，不能当作地面。

注意：补点是有限条件下的插值，不是证明未观测区域可通行。当前 Odin1 配置关闭 `patch_first_ring_to_baselink`，避免直接把近处盲区补到机器人脚下。

#### 第三步：连通分割，整理后续特征输入

`cloudSegmentation()` 在距离图上按上下左右邻接关系扩展非地面点簇。连接判断同时使用两点距离和扫描角间隔，不是“两个格子相邻就一定连接”。点簇必须满足点数/覆盖扫描行数条件，才进入有效分割结果。

有效非地面簇和经过抽稀的地面点共同组成 `segmented_cloud`，同时记录每点的距离、投影列号、地面标记及扫描行边界，供后面提取特征。

这里识别的是几何连续区域，不是语义分类：墙面可以形成有效非地面簇，但不会自动获得“这是墙”的标签。`segmented_cloud_pure` 在当前代码中有独立的地面提取阶段填充逻辑，不能直接等同于本步骤全部有效点簇。

#### 第四步：计算平滑度，再标记遮挡和不可靠点

`FeatureAssociation` 在特征提取前进行坐标及扫描时间相关处理；外部里程计模式还需匹配点云时间戳。随后 `calculateSmoothness()` 对分割点列中一个点前后各 5 个邻点计算：

```text
Dᵢ = 前 5 点距离之和 + 后 5 点距离之和 - 10 × 当前点距离
Cᵢ = Dᵢ²
```

代码将 `Cᵢ` 称为 curvature，但它是距离变化指标，不是拟合三维曲面得到的严格几何曲率。

- 平滑示例：中心距离 5 m，10 个邻点距离之和为 50.1 m，则 `D=0.1`、`C=0.01`。
- 边缘候选示例：中心距离 4.8 m，两侧距离分别为 `4.84、4.88、4.92、4.96、5.00 m`，邻点总和 49.2 m，则 `D=1.2`、`C=1.44`。

接着 `markOccludedPoints()` 标记不适合选作强特征的点：

- 相邻点投影列差小于 10，距离跳变超过 0.3 m 时，标记较远一侧边界附近的 6 个点。例如背景 5 m、前景 2 m，优先抑制背景侧的遮挡边界。
- 当前点与前后相邻点的距离差都超过自身距离的 2% 时，也标记该点。例如中心 4.8 m、左右均为 5 m，两侧差值 0.2 m 均大于 0.096 m；即便平滑度很大，也不能据此直接选成边缘。

这些是“不选作强特征”的标记，不等于删除所有原始点。后面的宽松表面收集并不再次按该标记过滤，所以不能把本步骤理解成所有输出都已彻底去除遮挡点。

#### 第五步：每条扫描行分成 6 段，先选边缘点

`extractFeatures()` 在每段内按 `Cᵢ` 从大到小选取，要求未被抑制、`Cᵢ > edge_threshold`，并且没有地面标记。

- 每段最多选 2 个强边缘点，进入 `laser_cloud_sharp`。
- 每段最多选 20 个较宽松边缘点，进入 `laser_cloud_less_sharp`，其中包含前述强边缘点。
- 选中后抑制附近最多前后各 5 个点，遇到较大的列间隔停止扩展，避免特征过度集中。

例如 `edge_threshold=0.1` 时，上一步 `C=1.44` 满足数值条件，但仍需通过分割、遮挡标记、地面标记和名额筛选。边缘是几何特征，不等于障碍物：墙角、柱边可能成为边缘，完整平墙内部则未必有很多边缘。

#### 第六步：再选严格平面点，最后收集宽松表面点

严格平面点按 `Cᵢ` 从小到大选择，当前代码同时要求：未被抑制、`Cᵢ < surf_threshold`、**地面标记为真**。每段最多取 4 个，输出到 `laser_cloud_flat`。例如 `surf_threshold=0.1` 时，`C=0.01` 通过数值条件，但仍需满足其他条件。

之后收集该段 `cloudLabel <= 0` 的点形成宽松表面集合，再逐扫描行做 0.2 m 体素降采样，输出到 `laser_cloud_less_flat`。这个收集条件包括已选的严格平面点和未被选为边缘的普通点，**不再次要求低平滑度、地面标记或未被遮挡标记**。

因此必须区分：

| 点集 | 当前代码的含义 | 墙面能否包含在内 |
|---|---|---|
| 地面 / `patched_ground` | 满足地面几何检查的观测及受约束补点 | 正常不应包含竖直墙面 |
| `sharp` / `less_sharp` | 通过筛选的非地面高距离变化特征 | 墙角、边界等可能进入 |
| `flat` | 通过严格筛选的低距离变化地面特征 | 正常墙面不能仅因平整而进入 |
| `less_flat` / 后续 surface 数据 | 未被选为边缘的较宽泛表面候选，经过降采样 | 可以包含墙面、地面及其他表面 |

这些集合不是互斥的“地面、边缘、平面”三分类。例如严格平面点也可能进入宽松表面集合；地面集合还可能含插值点。`surface` 也不代表程序已经为每块墙或地板拟合好了一个平面方程。

#### 第七步：结合位姿、关键帧筛选，保存并拼接

前面主要生成单帧局部几何信息。后续结合里程计/匹配得到位姿，进行关键帧筛选；满足保存条件时，分别保存 feature、ground、surface 点云及关键帧位姿，再按位姿拼接。

当前关键帧 `N_feature.pcd` 来自边缘特征链路，`N_ground.pcd` 来自补点地面链路，`N_surface.pcd` 来自宽松表面链路，随后可能继续降采样。它们不是三份相同点云换名字。详细文件及地图话题关系见第 7 节。

“仅使用外部里程计（跳过 scan-to-map）”跳过的是建图位姿的 scan-to-map 优化，并不跳过上述地面提取、分割和特征生成。原始帧数、被处理帧数、最终关键帧数也不是同一个数量。

### 6.2 本地补充：距离图与连通分割是什么

**距离图是按观测方向整理的点云表格，不是俯视地图。** 行代表垂直角，列代表水平角，格子记录该方向的距离 `r = sqrt(x² + y² + z²)`，并关联原来的 XYZ 点。这里的距离是到雷达的径向距离，不是点的高度，也不是相机坐标中的前向深度。

```text
                     水平角 →
                  左侧   正前方   右侧
垂直角   向上      5.2     5.0     5.2
  ↓      水平      5.1     2.0     5.1
         向下      3.0     2.1     3.0
```

上述数字单位为米；中间的 `2.0` 表示正前方、水平方向测到了距离 2 m 的物体。相邻格子只表示扫描方向相邻，不保证对应的三维点靠近。没有回波的格子保持无效，不能理解成距离为零或没有障碍；Odin1 未覆盖的后方扇区同样属于没有当前观测的区域。

| 数据 | 行、列的含义 | 格子记录的内容 |
|---|---|---|
| 距离图 | 垂直角、水平角 | 此方向的测量距离 |
| 二维占据地图 | 地面上的 X、Y 位置 | 此处的占据/空闲/未知状态 |

**连通分割是把可能属于同一连续表面的点分组。** 当前 `cloudSegmentation()` 对非地面候选执行四邻域扩展：选一个尚未分组的格子，检查上、下、左、右的邻居，根据两点距离及扫描角间隔判断是否连接；把能连接的点加入队列，继续扩展，直到形成完整点簇。随后检查点簇的点数及跨越扫描行数等条件。

例如 5 m 外的墙被 2 m 外的箱子局部遮挡，距离图局部可能是：

```text
5.0  5.1  5.0  5.0       墙  墙  墙  墙
5.0  2.0  2.1  5.0  →    墙  箱  箱  墙
5.1  2.0  2.1  5.1       墙  箱  箱  墙
5.0  5.0  5.1  5.0       墙  墙  墙  墙
```

这是几何分组示意，不保证这个小示例本身满足实际点簇筛选门槛。算法只产生簇编号，并不知道类别叫“墙”或“箱子”。从 5.0 m 到 5.1 m 可能连续，从 5.0 m 跳到 2.0 m 则可能断开；实际不是只比较一个固定距离差阈值，而是结合角间隔判断。

分割、遮挡检查、特征提取各有分工：分割判断哪些点连续；遮挡检查标记不可靠边界；特征提取再选择边缘和平面点。距离图和分割结果都不是最终的障碍地图。

### 6.3 本地补充：`segmented_cloud` 与 `segmented_cloud_pure` 的区别

WebUI 图层现以数据名显示：`mapground`、`mapcloud`、`segmented_cloud_pure`、`segmented_cloud`、`dGraph`，各自独立开关。导航时前两者订阅 `/map1/mapground`、`/map1/mapcloud`（不再以 `mapsurface` 充当“地图”图层）；建图时对应 `/lego_loam_ground`、`/lego_loam_map`，预览时读取 `ground.pcd`、`map.pcd`，具体来源见图层悬停提示。两个 segmented 图层使用消息时间戳对应的 TF 转到 `map`，无 TF 时等待，过期数据清空；预览不显示实时 segmented 点云。`dGraph` 仍只着色显示膨胀半径内的节点。`segmented_cloud` 的调试发布已恢复为有订阅者时发布，修改后需重编译 `lego_loam_bor` 并重启相关节点。

以下特指当前本地 `imageProjection.cpp`，不能仅根据话题名字套用其他 LeGO-LOAM 版本的解释。

| 对比 | `segmented_cloud` | `segmented_cloud_pure` |
|---|---|---|
| 主要填充阶段 | 连通分割之后 | 地面提取过程中 |
| 内容 | 有效非地面点簇和抽稀的地面点 | 部分未通过地面检查的点，以及地面判定角度范围外满足对应分支条件的点 |
| 地面 | 包含，有配套地面标记 | 偏向非地面候选，但不能保证完全没有地面 |
| 有效连通簇筛选 | 非地面点需经过该筛选，地面走独立保留逻辑 | 当前填充不以该筛选为前提 |
| 后续用途 | 提取边缘/平面特征，供里程计、建图或定位使用 | 当前导航配置的实时障碍感知输入 |

`segmented_cloud` 在 `cloudSegmentation()` 中收集点：保留有效非地面点簇，加入抽稀后的地面，排除无效值及无效点簇，并记录距离、列号和地面标记。因此它是特征提取的输入，不是最终边缘或平面特征集合。

`segmented_cloud_pure` 在地面提取函数中通过若干分支填充，例如端点法向不符合地面条件、连线坡度或高度差超限、位于地面判定角度范围外。**不是所有未被接受为地面的点都会加入它**：部分左右邻点缺失或高度连续性检查失败的分支直接跳过。因此不能写成 `segmented_cloud_pure = 原始点云 - 地面点云`。

源码末尾原有的“收集有效非地面连通簇并用簇编号设置 intensity”的 `pure` 填充代码块目前被注释；所以 `pure` 不代表“经过全部聚类筛选、完全去掉噪声的纯障碍点云”。

以墙面为例：墙点通过连通分割后可以进入 `segmented_cloud`，同一个点也可能因地面法向或坡度检查不通过而进入 `segmented_cloud_pure`。两者可以重叠，不是互斥集合。后者仍是障碍候选，必须经过后续感知处理，并不直接等于红色膨胀点。

## 7. 保存地图会生成什么

当前源码保存的典型结构为：

```text
地图目录/
├── map.pcd
├── ground.pcd
├── poses.pcd
├── edges.pcd
└── pcd/
    ├── 0_feature.pcd
    ├── 0_ground.pcd
    ├── 0_surface.pcd
    ├── 1_feature.pcd
    ├── 1_ground.pcd
    ├── 1_surface.pcd
    └── ...
```

| 文件 | 当前源码中的内容 | 对后续的作用 |
|---|---|---|
| `poses.pcd` | 每个关键帧的六自由度 `base_link` 位姿 | 地图服务器按位姿重建各关键帧点云，也是重定位地图坐标的基础。 |
| `edges.pcd` | 关键帧位姿图的边 | 描述相邻/回环约束，供位姿图检查和编辑。 |
| `pcd/N_feature.pcd` | 第 N 个关键帧的特征点 | 用于重建定位/匹配所需的特征地图。 |
| `pcd/N_ground.pcd` | 第 N 个关键帧的地面点 | 用于地面、可通行性、法向和导航地图生成。 |
| `pcd/N_surface.pcd` | 第 N 个关键帧的表面点 | 用于地图表面表达和下游规划。 |
| `map.pcd` | 当前源码汇总保存的特征地图 | 便于查看整体结构；不能代替全部关键帧文件。 |
| `ground.pcd` | 汇总的地面点 | 便于检查地面质量；会影响后续地面/可通行区域。 |

当前源码中 `map.pcd` 的汇总逻辑以特征点为主，地面和 surface 的汇总另有保存/处理路径。因此不能仅根据 `map.pcd` 判断地图是否完整，也不能只编辑一个汇总 PCD 就认为已彻底修复地图。地图服务器可能会根据 `poses.pcd` 和 `pcd/` 内的逐关键帧文件重新生成内容。

### 7.1 `map1/mapcloud`、`map1/mapground` 和 `segmented_cloud_pure` 从哪里来

这三个话题都是 `sensor_msgs/msg/PointCloud2`，但来源、坐标系和更新时间不同：

```text
建图保存的数据
├── poses.pcd + pcd/N_feature.pcd
│   └── dddmr_pg_map_server（节点名 map1）──> /map1/mapcloud
└── poses.pcd + pcd/N_ground.pcd
    └── dddmr_pg_map_server（节点名 map1）──> /map1/mapground

Odin1 当前帧 /astrall/lidar/points
└── mcl_feature 内的 ImageProjection ──────> /segmented_cloud_pure
```

#### `/map1/mapcloud`：已保存地图中的静态特征/障碍结构

- 建图时，地图优化把第 `N` 个关键帧的特征点保存为 `pcd/N_feature.pcd`，把对应的六自由度 `base_link` 位姿保存到 `poses.pcd`。
- 导航启动时，`dddmr_pg_map_server` 节点读取这些文件，用 `poses.pcd` 中的位姿把每个关键帧从 `base_link` 坐标变换到 `map` 坐标并累加。
- 当前源码随后使用 `0.2 m` 聚类容差做欧式聚类，删除少于 10 个点的孤立簇，移除 NaN，再按 `complete_map_voxel_size` 体素降采样。
- 地图服务器使用私有话题 `~/mapcloud`；当前节点名是 `map1`，所以完整话题名解析为 `/map1/mapcloud`。
- 它是加载地图时生成的**静态特征地图**，机器人移动不会使其随当前扫描变化。Astrall 的全局和局部静态感知层把它作为已有障碍/环境结构输入；如果建图时把机械狗自身、地面噪声或拖影写入了特征关键帧，这些错误也会固定出现在后续障碍与膨胀结果中。

它不是简单读取汇总文件 `map.pcd` 后直接发布。只修改 `map.pcd`，但不修改相应的 `pcd/N_feature.pcd`，重新加载地图后污染点仍可能再次出现。

#### `/map1/mapground`：已保存地图中的静态地面

- 建图的地面提取和补地面处理生成关键帧地面点，地图优化将其保存为 `pcd/N_ground.pcd`。
- 导航启动时，同一个 `dddmr_pg_map_server` 使用 `poses.pcd` 把所有地面关键帧变换到 `map` 坐标，移除 NaN，并按 `complete_map_voxel_size` 降采样。
- 私有话题 `~/mapground` 在节点 `map1` 下解析为 `/map1/mapground`。
- 它同样是加载后基本不变的**静态地面地图**，用于建立可通行地面节点、连接关系以及到静态障碍的距离。地面缺失会造成可规划区域断裂；墙面、机身或悬空点误入地面，则可能产生错误的可通行区域或异常膨胀点。

它也不是只从汇总的 `ground.pcd` 直接发布；真正参与重建的是 `poses.pcd` 和每个 `pcd/N_ground.pcd`。

#### `/segmented_cloud_pure`：Odin1 当前扫描产生的实时非地面候选点

- 导航阶段的 `mcl_feature` 可执行程序内部创建 `ImageProjection` 节点，并把内部输入 `lslidar_point_cloud` 重映射到 `/astrall/lidar/points`。
- 每个被接受的雷达帧会先去除无效点，根据 YAML 中的扫描行列、距离、忽略视场和地面视场等参数投影并提取地面。
- 当前 `sendinn` 源码主要在地面补片/地面移除阶段把未被接受为地面的点加入 `_segmented_cloud_pure`，然后使用输入点云原有的时间戳和 `header.frame_id` 发布 `/segmented_cloud_pure`。
- 源码中“把有效连通域标签重新填入 `_segmented_cloud_pure`”的后续代码块目前被注释。因此在这个版本中，它更准确的含义是**实时非地面/障碍候选点**，不能仅凭话题名理解成“已经完成全部聚类筛选的纯障碍点”。
- Astrall 的全局和局部雷达感知层订阅它，通过 TF 变换到 `map`，用于实时标记和清除动态障碍代价。它会随 Odin1 当前帧变化；Odin1 未扫描到的后方扇区不应被当作当前帧已确认的自由空间。

三者在感知和规划中的组合关系是：

| 话题 | 静态/实时 | 主要内容 | 对导航的主要作用 |
|---|---|---|---|
| `/map1/mapcloud` | 静态 | 保存地图中的特征和结构点 | 提供已有静态障碍，计算静态障碍距离和代价。 |
| `/map1/mapground` | 静态 | 保存地图中的地面点 | 构建可通行地面图和规划搜索节点。 |
| `/segmented_cloud_pure` | 实时 | Odin1 当前帧的非地面/障碍候选点 | 标记或清除当前观测范围内的动态障碍。 |

最终看到的红色或橙色膨胀点通常不是这三个话题中的原始点，而是感知模块把静态 `mapcloud` 和实时 `segmented_cloud_pure` 施加到静态 `mapground` 地面图后，生成的带障碍距离/代价的 `perception_3d_ros/dGraph` 节点。

#### 本地补充：定位、全局规划与局部避障分别使用什么

| 阶段 | 主要数据 | 作用 |
|---|---|---|
| MCL 定位 | 实时边缘/平面特征、静态特征与地面子地图、里程计 | 估计机器人在地图中的位置和朝向 |
| 静态障碍感知 | `map1/mapcloud` | 提供建图时保存的静态障碍结构 |
| 地面构图 | `map1/mapground` | 提供地面节点和连接关系，再由感知判断可通行性 |
| 实时障碍感知 | `segmented_cloud_pure` | 提供当前雷达的非地面/障碍候选 |
| 全局规划 | 感知维护的地面图及障碍距离/代价 | 搜索到目标的路线 |
| 局部避障 | 静态与实时障碍观测、机器人碰撞外形、候选轨迹 | 检查预测运动是否碰撞，选择可执行速度 |

```text
当前雷达点云
├── segmented_cloud → 边缘/平面特征 → 与静态子地图匹配 → MCL 位姿
└── segmented_cloud_pure → 实时障碍感知
                           ├── 影响地面图的障碍距离/代价 → 全局路径
                           └── 提供局部障碍观测 → 轨迹碰撞检查

map1/mapground → 地面节点及连接关系
map1/mapcloud  → 静态障碍约束，同时可提供局部碰撞观测
```

当前 `robot/astrall/astrall_dddmr_bringup/config/odin1_navigation.yaml` 的全局、局部感知都订阅 `segmented_cloud_pure`，同时加载静态地图层。局部静态层还配置了 `collision_observation: true`，把静态 `mapcloud` 纳入局部轨迹碰撞评分。因此避障不只是看当前扫描，全局路径也可能受实时障碍影响。

需要区分以下故障来源：

- 地面提取错误会影响 `mapground`，可能导致分层、路径断开或起点投影错误。
- 当前静态层订阅的是 `mapcloud`，不是 `mapsurface`。界面 surface 显示了一面墙，不代表静态碰撞输入中一定有足够墙体点。
- `segmented_cloud_pure` 漏掉障碍时，调大后续代价不能凭空恢复缺失的观测。
- 定位或 TF 错误时，即使障碍点完整，也可能被变换到错误的位置。
- 红色/橙色膨胀点是地面节点上的障碍距离/代价可视化，不是原始回波；看到或看不到红点，都不能单独证明局部碰撞检查是否正常。

本节描述当前源码配置的数据链路，不代表运行时已经验证成功。现场仍需确认实际加载的配置、话题更新时间、TF、感知输出和碰撞检查结果。WebUI 勾选显示仅控制可视化，不等于启用或验证避障。

### 7.2 `/perception_3d_ros/dGraph` 如何生成

`dGraph` 可以理解为一张“以地面点为节点、为每个节点记录最近危险距离”的图。它不是原始障碍点云，也不是传统二维占据栅格。

```text
/map1/mapground ───────────────> 地面节点 i（规划候选位置）
                                        │
/map1/mapcloud ──> 静态地图层 ──────────┤
                                        ├─> dGraph[i] = 各感知层给出的最小值
/segmented_cloud_pure ─> 实时雷达层 ────┤
                                        │
禁行区等其他插件 ────────────────────────┘

发布点：(x, y, z) = mapground[i]
        intensity = dGraph[i]
```

处理流程如下：

1. `/map1/mapground` 中每个点按原有顺序成为一个地面节点。
2. 每个感知插件都有一份同样大小的动态图，初始值为 `max_obstacle_distance`；当前配置是 `9999.0 m`，表示该层暂未发现邻近危险。
3. 静态地图层根据 `/map1/mapcloud` 给地面节点写入静态危险值。
4. 实时雷达层将 `/segmented_cloud_pure` 变换到 `map`，经过有效视场、高度、聚类和感知窗口处理后，将障碍簇投影到地面，并计算附近地面节点到最近投影点的距离。
5. 同一个节点如果被多个障碍或多个插件影响，只保留更小、更危险的距离：

   ```text
   dGraph[i] = min(d_static[i], d_lidar[i], d_no_entry[i], ...)
   ```

6. `/perception_3d_ros/dGraph` 会发布**全部地面节点**，并将合并后的值写入 `PointXYZI.intensity`。当前全局配置的 `dgraph_publish_frequency` 是 `1.0 Hz`。

当前实时雷达层的关键阈值为：

```yaml
inscribed_radius: 0.45          # 小于该距离为硬碰撞区域
inflation_radius: 0.65          # 只在该范围内给动态地面节点写入障碍距离
inflation_descending_rate: 2.0  # 规划代价随距离衰减的速度
perception_window_size: 5.0     # 当前机器人附近的动态标记/清除窗口
```

在实际观测区域内，实时层会先把相关节点恢复为远距离值，再用当前障碍重新标记。Odin1 是前向扇形雷达，未观测的后方区域不能视为已经清空。

### 7.3 红色、橙色和“代价”的准确含义

WebUI 不会把 `dGraph` 的所有节点都画出来，而是只显示 `intensity <= inflation_radius` 的节点：

| `dGraph.intensity` | WebUI 显示 | 规划含义 |
|---:|---|---|
| `< 0.45 m` | 红色 | 地面节点距离危险源小于机械狗硬碰撞半径，规划器禁止通过。 |
| `0.45～0.65 m` | 橙色 | 软膨胀区域，仍可能参与搜索，但规划器会尽量绕开。 |
| `> 0.65 m` | 不显示 | 当前膨胀范围之外。 |
| 接近 `9999 m` | 不显示 | 当前感知层没有为该节点找到邻近危险。 |

因此，红色或橙色点的坐标来自 `mapground`，表示“机械狗中心走到这个地面位置是否安全”；它不是墙壁、桌腿等障碍物自身的原始坐标。

`dGraph.intensity` 在主要感知层中表示距离，不等于最终路径总代价。对于没有进入硬碰撞区的节点，当前全局规划器根据下式形成障碍附加惩罚：

```text
障碍惩罚 = exp(-inflation_descending_rate ×
               (dGraph 距离 - inscribed_radius))
```

当前 `inflation_descending_rate=2.0`、`inscribed_radius=0.45 m` 时，距离越接近 `0.45 m`，惩罚越大。规划器还会叠加实际路程、转弯惩罚、地面连接/边缘惩罚等，所以最终路径代价不只由 `dGraph` 决定。

### 7.4 当前静态层为何检查地面节点上方，以及轨迹红点排查

当前 `sendinn` 分支的静态层需要把空间中的 `/map1/mapcloud` 障碍结构关联到 `/map1/mapground` 的规划节点。对每个地面节点，它执行一套保守的启发式判断：

1. 先在 `static_imposing_radius` 内取得候选静态地图点；当前 Astrall 全局配置为 `5.0 m`。这只是候选搜索范围，后面仍会继续裁剪。
2. 保留相对该地面节点高 `0.1～1.0 m` 的点，以排除地面本身及其厚度噪声，并关注可能碰撞机械狗主体的高度带。
3. 再限制到地图坐标轴方向的 `x/y ±0.5 m` 方框内；它是固定轴对齐方框，不是随机械狗朝向旋转的精确包围盒。
4. 如果剩余 `/map1/mapcloud` 点超过 10 个，就将该地面节点的静态 `dGraph` 值写成固定的 `0.25 m`。

这里的 `0.25 m` 是**固定危险标记**，不是计算得到的真实最近障碍距离。由于当前 `inscribed_radius=0.45 m`，该节点会直接成为红色不可通行节点。`>10` 用于过滤少量孤立噪声，但效果会受点云密度、关键帧重复程度和体素大小影响。

这套规则的设计目标是判断“机械狗中心能否安全站在这个地面节点上”，但存在以下局限：

- 固定点数阈值对不同雷达和点云密度并不等价。
- `x/y ±0.5 m` 比 Astrall 当前约 `0.36 m` 的机身半宽更保守。
- 固定 `0.25 m` 无法反映真实障碍距离。
- 地图轴对齐方框无法准确表达机械狗朝向。
- 被错误保存到 `mapcloud` 的地面点、运动拖影或重叠点，也可能满足“超过 10 点”而成为红色节点。

当前 Odin1 安装为向前扫描，若已通过实机点云确认有效扇区不会扫到机械狗自身，应将“自身回波”从首要嫌疑中移除。轨迹附近仍出现红点时，优先检查：

1. 地面点是否误入 `/map1/mapcloud`。
2. 行走俯仰、横滚、上下起伏是否因点云与里程计时间不同步形成高于地面 `0.1 m` 以上的拖影层。
3. `base_link -> lidar_link` 外参是否准确，是否存在重复 TF 发布者。
4. `num_vertical_scans`、`num_horizontal_scans`、垂直角范围和 `scan_period` 是否符合实际 Odin1 点云组织。
5. `ground_fov_*`、`ground_positive/negative_*`、坡度和高度差阈值是否把前方地面稳定分入 ground。
6. 红点是否只是靠近真实墙面或立柱；若红点完整沿轨迹中心出现，更像地面误分类或运动拖影。

诊断时应分别显示：

```text
/map1/mapground
/map1/mapcloud
/map/dGraph
/lidar/dGraph
/perception_3d_ros/dGraph
```

- `/map/dGraph` 已出现异常红点：问题来自保存的静态地图或静态判定规则。
- `/map/dGraph` 正常而 `/lidar/dGraph` 出现异常：问题来自实时 `/segmented_cloud_pure`、TF 或动态标记/清除。
- `/map1/mapcloud` 在地面上方形成平行薄层：优先排查时间同步、外参和运动畸变。
- `/map1/mapcloud` 与地面混在一起：优先排查地面分割和扫描模型。

WebUI 导航模式下显示的蓝色“地图”目前是 `/map1/mapsurface`，而静态膨胀实际使用 `/map1/mapcloud`。因此蓝色地图看起来正常，并不能证明用于静态障碍判断的特征地图没有污染。不要在确认输入点云之前，仅通过提高 `>10` 的点数阈值、减小碰撞半径或隐藏红点来规避问题。

## 8. 地图质量对定位和导航的影响

| 建图问题 | 定位影响 | 导航影响 |
|---|---|---|
| 地面被误判为障碍/特征 | MCL 匹配不稳定，移动几步后不确定度增大 | 轨迹附近出现红色膨胀点，规划器认为道路不可通行。 |
| 机械狗自身回波进入地图 | 重定位可能匹配到“随机器人移动的假结构” | 机器人包围盒内或建图轨迹上形成连续障碍带。 |
| 时间戳或 TF 错位 | 同一物体出现多层、拖影，关键帧位置漂移 | 静态障碍地图变厚，安全距离被过度占用。 |
| Odin1 后方扇区未观测 | 可用于匹配的几何信息不足，朝向更易模糊 | 后方应保持未知；不能依赖雷达清除后方动态障碍。 |
| 关键帧过密 | 地图重复点多，加载和定位更慢 | 感知/规划计算量增加，障碍可能显得更厚。 |
| 关键帧过稀 | 几何结构不连续，重定位可匹配区域减少 | 地面和通道断裂，全局规划可能失败。 |
| 未闭环或闭环错误 | 前者保留累计漂移，后者导致地图整体变形/突跳 | 规划路径与真实环境错位，严重时不可安全使用。 |

重定位主要依据实时点云与保存地图的几何匹配，并结合初始位姿分布和运动更新。初始位姿最好位于建图时实际走过、结构明显且与当时朝向相近的位置；不要求完全重合，但 Odin1 只有前方扇形视场，可见特征较少，因此初始位置和朝向越接近正确值，越容易收敛。

## 9. 建图验收清单

在把地图交给 MCL 和导航前，至少检查：

- 静止时点云、`ground_cloud` 和特征点不应持续漂移或分裂成多层。
- 雷达近场没有机械狗腿、机身、线缆等自身回波；若有，应先做机身裁剪。
- 地面点连续但不过厚，墙、路沿和真实障碍没有被大面积归入地面。
- 建图轨迹附近没有跟随机身留下的连续障碍点带。
- `poses.pcd` 的轨迹与真实路线一致，没有突然跳变或不合理折返。
- 回到走过区域时点云能够对齐；启用回环后地图没有突然被错误拉伸。
- 保存目录包含 `poses.pcd`、`edges.pcd` 以及完整的 `pcd/N_*.pcd` 文件组。
- 在启动导航前，先预览定位地图、地面地图和膨胀障碍地图，再允许机器人运动。

## 10. 本地补充：起步旋转后的制动与轨迹候选

Astrall 配置启用两个本地新增开关（通用代码默认关闭）：

- `trajectory_generators.omni_drive_simple.sample_braking_commands: true`：将每个允许零速度的轴的候选指令区间扩展到零，避免实测速度偏大时只能继续同方向运动。`VelocityIterator` 会包含零，三个轴的组合覆盖直行、单轴制动和全停。全停候选不受最小运动速度过滤，但仍经过轨迹碰撞评分。
- `p2p_move_base.stop_after_heading_alignment: true`：航向对准后持续发送零速度；要求三个新鲜且不同的里程计样本满足平移速度不超过 `0.03 m/s`、角速度绝对值不超过 `0.05 rad/s`，再重新确认航向。里程计最大允许年龄 `0.5 s`，五秒未确认停稳则中止任务。这些是本地保守判据，不是厂商标称精度。

零速度指令不等于预测瞬间停止：候选轨迹仍从实测速度出发，按配置加减速度积分；反向运动先减速到零。该模型还需要与真实执行响应核对，编译通过不能证明机械狗能安全沿路径到达。不要通过放宽碰撞或定位门限补偿跟踪问题。

## 11. 本地补充：执行期安全门禁（尚需离线及实机验收）

- Astrall 的局部静态层启用 `map.collision_observation: true`，将静态 `mapcloud` 的快照汇入局部碰撞评分，不再只依靠实时点云。地图和地面未初始化时禁止通过感知就绪检查。
- 局部雷达过滤后的实时障碍点少于 5 个时禁止运动；汇总碰撞点不足 5 个也拒绝候选。静态地图不能替代实时感知就绪。雷达同时检查处理间隔和输入消息年龄，局部评分读取加锁复制的点云快照。这是保守停车策略：无障碍的空旷环境也可能停车，点数达到阈值仍不代表盲区安全。
- `p2p_move_base.localization_timeout: 0.5` 要求最近通过匹配的 `mcl_pose` 足够新鲜，同时检查 XY/yaw 方差。失效后发零速度、中止目标并取消恢复行为；定位恢复不会自动恢复该目标。门禁在控制循环检查，不等同于独立硬件急停。
- 执行期 `localization_xy_std_max: 0.15`、`localization_yaw_std_max: 0.20` 独立于 WebUI 的准入门限；Web 修改门限不会自动修改这两个执行期参数。

仍待确认：碰撞盒必须覆盖真实机身/腿部的运动包络；当前 `base_link` 下 `z=0～0.6 m` 未经尺寸验收。路径阻塞检查的 `check_radius` 是路径点周围的三维球，不是机器人外形；Web 中过小的距离不能提供全身保护。上述修改未调整其数值，也未验证真实制动距离或 Odin1 盲区覆盖。不要将编译通过当作防撞验收通过。

## 12. 本地补充：点云话题总览与建图、定位、导航用途

本节汇总前述讨论，范围为当前 Astrall/Odin1 建图、定位、感知和 Web 链路，以及相关调试输出，不是仓库所有可选机器人配置的完整 ROS 接口清单。依据当前本地源码与配置整理；“有发布器”不等于“正在发布”，也不等于“算法正在使用”。话题可能受 namespace/remap 影响，现场名称、消息频率和订阅者应以 ROS 图为准。

### 12.1 最重要的区别

- `segmented_cloud` 是当前帧特征提取的原材料；内部数据对定位有用，额外发布同名话题主要为了观察，停止显示不影响内部处理。
- `segmented_cloud_pure` 是当前实时障碍感知的候选点输入，不保证包含所有非地面点，也不是最终膨胀图。
- `mapcloud` 是保存的 feature/边缘特征地图；当前静态障碍层使用它。
- `mapsurface` 是保存的宽松表面点地图，通常能显示更丰富的墙面和环境表面；当前主要用于可视化和地图质量检查，并未作为静态障碍层输入。
- `mapground` 是保存的地面点地图，用来构建地面图，并提供定位所需的地面几何。绿色地面点不等于已经验证无碰撞。
- `dGraph` 是地面节点及其危险距离值，不是原始障碍点云；颜色由显示端根据距离阈值生成。

### 12.2 传感器、投影和分割话题

以下点云话题的消息类型均为 `sensor_msgs/msg/PointCloud2`。输入局部云通常为 `lidar_link`，实际以消息 `header.frame_id` 为准，不要根据话题名字猜坐标系。

| 话题 | 来源 / 内容 | 用途与当前注意事项 |
|---|---|---|
| `/astrall/lidar/points` | Astrall 桥接输出的雷达局部观测 | 建图、MCL 特征前端的实际雷达输入；Web RGB 扫描也使用它。不是已经拼好的静态地图。 |
| `/astrall/lidar/points_base` | 转到 `base_link` 的点云 | 检查点云与机身、碰撞盒的相对关系；不是当前默认建图输入。是否发布取决于桥接配置。 |
| `/astrall/lidar/points_odom` | 转到里程计坐标的点云 | 检查配准和累积效果；不能直接替代算法要求的雷达局部输入。是否发布取决于桥接配置。 |
| `lslidar_point_cloud` | DDDMR 前端内部订阅名 | 当前通过 remap 接到 `/astrall/lidar/points`，不是另一台雷达或另一份必然存在的点云。 |
| `/full_cloud_info` | 距离图对应的投影点云及附加信息 | 投影调试；当前创建了发布器，但发布调用被注释。 |
| `/ground_cloud` | 当前帧补点并降采样后的地面 | 检查单帧地面判断；当前发布的是 `ds_patched_ground_`，不是整张 `mapground`。 |
| `/segmented_cloud` | 有效非地面点簇和抽稀地面点 | 内部进入特征提取；当前有订阅者时额外发布，供 Web/调试观察。 |
| `/segmented_cloud_pure` | 地面提取阶段若干分支保留的非地面/障碍候选 | 当前全局、局部雷达感知层的输入；处理后的结果才参与障碍标记和碰撞检查。 |
| `/outlier_cloud` | 连通分割拒绝的部分点 | 离群/分割调试；当前投影节点的发布调用被注释。内部离群数据仍可能传向后续阶段。 |

`/segmented_cloud_info` 不是 PointCloud2，而是 `cloud_msgs/msg/CloudInfo`：保存扫描行边界、地面标记、列号、距离等配套信息。当前前端还通过进程内 `Channel<ProjectionOut>` 传递点云及信息，不依赖 Web 对 `/segmented_cloud` 的订阅来驱动算法。

### 12.3 特征提取与建图输出

| 话题 | 内容 | 用途 / 限制 |
|---|---|---|
| `/laser_cloud_sharp` | 少量强边缘特征 | 当前 MCL 四路同步输入之一，但其点云解码未用于当前评分；不能因此随意停止发布，否则可能影响同步回调。 |
| `/laser_cloud_less_sharp` | 较宽松边缘特征，包含 sharp | 当前 MCL 实际使用的边缘输入；建图的 feature 数据也来自边缘特征链路。 |
| `/laser_cloud_flat` | 严格低平滑度且具有地面标记的特征 | 当前 MCL 实际使用的地面/平面特征，之后还会降采样。 |
| `/laser_cloud_less_flat` | 未被选为边缘的较宽泛表面候选，降采样后输出 | 建图 surface 链路；当前 MCL 同步订阅，但对应解码未用于评分。不是严格拟合出的纯平面集合。 |
| `/laser_cloud_corner_last` | 特征关联处理后的边缘云 | 后续建图与调试相关输出；内部建图传输还使用 `AssociationOut` 通道。 |
| `/laser_cloud_surf_last` | 特征关联处理后的表面云 | 后续建图表面数据及调试；与保存后发布的 `mapsurface` 不是同一时刻、同一坐标的对象。 |
| `/outlier_cloud_last` | 特征关联处理后的离群云 | 后续处理与离群诊断；不要与当前未发布的 `/outlier_cloud` 混淆。 |
| `/lego_loam_map` | 建图可视化模块输出的累积地图 | Web 建图模式下 `mapcloud` 图层的数据来源；不等于导航地图服务器话题。 |
| `/lego_loam_ground` | 建图可视化模块输出的累积地面 | Web 建图模式下 `mapground` 图层的数据来源。 |
| `/lego_loam_ground_edge` | 建图地面边缘可视化相关输出 | 地面边界诊断；是否有数据取决于可视化模块的实际发布分支。 |
| `/cloud_keypose_6d` | 以点云形式携带关键帧六自由度位姿 | 位姿图/关键帧诊断，不是环境回波点。 |
| `/laser_cloud_surround` | 周围关键帧地图相关调试接口 | 检查局部地图；声明发布器不保证当前执行分支发送数据。 |
| `/recent_corner_cloud`、`/recent_surf_cloud` | 近期边缘、表面关键帧相关调试接口 | 检查 scan-to-map 周围地图；依赖实际发布分支。 |
| `/optimized_corner_cloud`、`/optimized_surf_cloud` | 优化后的边缘、表面相关调试接口 | 检查优化对齐效果；不是独立的导航障碍输入。 |
| `/selected_lm_cloud` | 优化选点相关调试接口 | 检查匹配/优化使用的点；不保证每种建图模式都有输出。 |
| `/loopclosure_target_cloud`、`/corrected_cloud` | 回环目标与校正点云相关输出 | 回环诊断；关闭回环或未进入相应分支时可能没有输出。 |

“仅使用外部里程计”跳过 scan-to-map 位姿优化，不跳过地面和特征生成。建图内部 surface 特征参与优化的可能性，也不等于导航时 `/map1/mapsurface` 被 MCL 使用。

### 12.4 已保存地图与 MCL 子地图

| 话题 | 来源 | 当前用途 |
|---|---|---|
| `/map1/mapcloud` | `poses.pcd` + `pcd/N_feature.pcd`，按位姿拼接并过滤/降采样 | 完整静态特征地图；当前静态感知层输入，Web `mapcloud` 图层。 |
| `/map1/mapsurface` | `poses.pcd` + `pcd/N_surface.pcd` | 完整表面地图；Web 独立 `mapsurface` 图层，检查墙面重影、表面错位等。当前未加入 MCL 匹配子地图，也不是静态层配置输入。 |
| `/map1/mapground` | `poses.pcd` + `pcd/N_ground.pcd` | 完整地面地图；地面图构建、静态/动态危险值计算的节点基础，Web `mapground` 图层。 |
| `/sub_mapcloud` | MCL 附近关键帧组成的特征子地图 | 展示当前匹配区域；评分使用对应内部 KD-tree，不是靠订阅此调试输出。 |
| `/sub_mapground` | MCL 附近关键帧组成的地面子地图 | 地面匹配与法向支持对应的子地图输出。 |
| `/sub_mapcloud_warmup`、`/sub_mapground_warmup` | 预加载的下一组特征/地面子地图 | 检查子地图预热和切换，不代表已经成为当前评分地图。 |
| `/euclidean_cluster_extraction` | MCL 对实时边缘特征聚类、设置权重后的点云 | 定位预处理诊断；当前发布在 `base_link`，intensity 承载算法权重，不是原始反射强度。 |
| `/ground_normal` | MCL 创建的地面法向相关 PointCloud2 接口 | 当前主定位源码中未见对应发布调用，不能仅凭名称认为一定能收到法向点云。 |

仓库还保留独立 `pcl_publisher` 的 `mapcloud` / `mapground` 接口，但当前 Astrall 位姿图导航使用的是 `map1` 地图服务器，二者不可混淆。MCL 按附近关键帧构建自己的子地图，不是直接拿 Web 显示的整张 `mapsurface` 来评分。

### 12.5 感知与膨胀图输出

| 话题 / 名称模式 | 内容与用途 |
|---|---|
| `/perception_3d_ros/dGraph` | 各层融合后的地面节点；XYZ 来自地面图，intensity 是各层危险距离值的最小值。Web `dGraph` 图层从这里取数据。 |
| `map/dGraph` | StaticLayer 的单层地面危险值调试输出；名称由插件名 `map` 拼接，不等于融合总图。命名空间以现场为准。 |
| `<感知节点>/lidar/current_observation` | 雷达层的当前观测调试输出，检查真正进入感知链路的点。 |
| `<感知节点>/lidar/lethal` | 雷达层危险点相关调试输出，不等于整张地面代价图。 |
| `<全局感知节点>/lidar/current_segmentation` | 感知层聚类结果，注意不是前端 `/segmented_cloud`。 |
| `<全局感知节点>/lidar/current_projected` | 感知层当前投影结果，检查障碍向地面施加影响的中间过程。 |
| `<全局感知节点>/lidar/marked_voxel` | 障碍体素标记可视化。 |
| `<全局感知节点>/lidar/current_window_marking` | 当前感知窗口内标记可视化。 |
| `<全局感知节点>/lidar/global_marking` | 全局标记可视化，受相关发布开关控制。 |
| `<全局感知节点>/lidar/dGraph` | 雷达单层地面危险值；不是静态层与动态层融合后的总图。 |
| `no_entry_pc_zones`、`speed_limit_pc_zones` | 可选禁行区、限速区插件的点云可视化；当前 Astrall 插件列表未启用这两层。 |

当前感知节点名通常为 `perception_3d_global` 和 `perception_3d_local`。上述调试话题依赖全局/局部模式、订阅者及可视化开关，不保证同时有数据。规划器还直接使用进程内感知数据，不能把“没有发布调试点云”直接等同于“算法没有感知数据”。

### 12.6 定位如何使用 segmented_cloud，为什么运动后仍可能漂移

```text
当前雷达 → 内部 segmented_cloud → 特征筛选
                                  ├─ less_sharp → 静态特征子地图
                                  └─ flat       → 静态地面子地图
里程计 → 预测各候选位姿 → 将特征变换到 map → 最近邻距离评分
                                       → 更新粒子权重、位姿和不确定度
```

当前 MCL 同步四路特征消息，但主要解码 `less_sharp` 和 `flat` 参与评分。对每个粒子位姿 `R,t`，将当前机器人坐标中的特征 `p` 变为 `R*p+t`，再查对应地图最近邻；地面支持不足时，flat 分支退回特征地图搜索。匹配比例是距离满足 `likelihood.match_dist_min` 的评分特征占比，分母不是原始整帧回波数。评分还包含簇权重、地面支持等项。

例如局部边缘点 `(2,0)` 对应地图墙角 `(5,1)`：候选位姿 `(3,1, yaw=0)` 会把它放到 `(5,1)`，距离为 0；候选 `(3.5,1, yaw=0)` 则放到 `(5.5,1)`，距离为 0.5 m。多个点共同决定哪种位姿更可信，而不是一个点对上就算定位成功。

起点定位正确不保证运动后一直正确。里程计误差、点云时间/TF 错配、机身起伏、地图重影、重复走廊和前向有限视场，都可能让后续匹配退化或误匹配。应同时检查扫描与地图对齐、匹配比例、位姿更新时间、位姿连续性和不确定度；“协方差小/已收敛”仅表示粒子集中，不保证集中在真实位置。不能靠放宽门槛来替代定位修复。

### 12.7 Web 图层与非点云话题的边界

当前六个点云开关为 `mapground`、`mapcloud`、`mapsurface`、`segmented_cloud_pure`、`segmented_cloud`、`dGraph`；RGB 扫描另显原始彩色观测。`mapsurface` 在加载导航地图后显示，建图及文件预览不拿其他点云冒充它。勾选只是可视化操作，不会把一个话题接入定位/避障，也不会改变算法的启用状态。

`mapcloud` 与 `mapsurface` 可以重叠但用途不同：完整表面可见不代表静态避障输入足够。当前静态层用 `mapcloud`，实时层用 `segmented_cloud_pure`，在 `mapground` 上形成危险距离；局部碰撞评分还使用障碍观测和机器人碰撞外形，不能只看红点判断是否安全。

以下常一起出现，但**不是 PointCloud2 点云话题**：`/mcl_pose`（位姿及协方差）、`/particles`（粒子位姿数组）、`/astrall/base/odometry` 和 `/laser_odom_to_init`（里程计）、`/global_path` 和 `/awared_global_path`（路径）、`/tf` 和 `/tf_static`（坐标变换）、`/cmd_vel`（速度指令）。它们分别解决“在哪、怎样移动、往哪走、如何变换和执行”的问题。

可在已启动的环境中只读核验话题，不会发送运动指令：

```bash
ros2 topic list -t
ros2 topic info /segmented_cloud -v
ros2 topic info /map1/mapsurface -v
ros2 topic hz /segmented_cloud_pure
```

静态地图往往加载时发布并通过 transient-local 保留，不要求像雷达一样持续高频发布。核对源码可从 `imageProjection.cpp`、`featureAssociation.cpp`、`mapOptimization.cpp`、`lego_loam_map_visualization.hpp`、`dddmr_pg_map_server.cpp`、`mcl_3dl.cpp`、`sub_maps.cpp`、`lidar_measurement_model_likelihood.cpp`、`static_layer.cpp`、`multilayer_spinning_lidar.cpp` 和 Astrall `web_ui/server.py` 入手。

## 13. 本地补充：Web 地图体素参数与沿墙多排红点

Web 导航面板增加 `complete_map_voxel_size`，单位米，默认 0.2，界面允许 0.05～1.0。数值会作为加载参数写入临时导航 YAML，不修改模板或原始 PCD；导航运行期间锁定，需要结束当前任务、修改后重新加载地图，不属于“应用避障与膨胀参数”的热更新项目。浏览器保存上次输入，导航状态显示本次实际请求值，启动日志记录地图体素。

这个参数同时影响地图服务器发布的 `mapcloud`、`mapsurface`、`mapground`，不是单纯的 Web 显示稀疏度。较小体素既增加地面图节点，也增加静态层方框内的特征点。当前静态层固定点数门槛不会随体素大小自动归一化。

一面墙旁边出现多排红点，不代表检测出了多面墙：红点位置来自附近地面节点，同一段墙可以使多个节点危险。当前静态判定在节点上方 0.1～1.0 m、XY 各 ±0.5 m 的方框中统计 `mapcloud` 点，超过 10 个便写入固定值 0.25；它不是实际测得的障碍净空。因此同时存在“危险地带有多排地面采样”和“固定计数/固定距离造成过度标记”两种可能，不能通过减少显示点数来修复后者。

对 `astrall_company/2026_09_16_12_32_49` 做了一次离线近似检查：按 `poses.pcd` 的 XYZ/RPY 将 297 个 feature、ground 关键帧变换到地图，合计 60439 个特征点、172583 个地面点，再用体素质心降采样并统计上述方框条件：

| 体素 | 特征点数 | 地面节点数 | 方框中超过 10 个点的节点数 |
|---|---:|---:|---:|
| 0.4 m | 2084 | 459 | 57 |
| 0.2 m | 6275 | 1865 | 1023 |

0.2 m 组中，有 58 个满足计数条件的节点，到方框内最近上方特征点的水平距离仍大于 0.45 m（该子集最大约 0.507 m），但固定计数规则会赋予 0.25。该统计只说明计数机制对密度敏感、固定危险值不等于真实距离，不能据此认定这些位置对机器人安全。

限制：本次未连接到本机 8088 Web 服务，未获取实时 dGraph；离线统计没有复现地图服务器的全部聚类过滤、small_gicp 的所有实现细节、静态层的地面邻域前置条件或动态层融合，因而不是运行时红点数，也不能证明具体某个红点误报。本次未改变静态判定、碰撞半径或导航安全门槛。

### 静态障碍判定参数（回退后的本地修改）

上一轮“最近水平距离”算法已回退，恢复原来的三维搜索、方框筛选和点数判定。默认保留原行为：地面邻域至少 5 点，搜索球半径 5 m，地面上方 0.1～1.0 m，X/Y 各 ±0.5 m，至少 11 个点时赋值 0.25 m（固定危险值，不是实测墙距）。

Web 导航页新增“静态地图障碍判定”，与 `odin1_navigation.yaml` 的全局感知 `map` 配置对应：

| 配置项 | 含义 |
|---|---|
| `static_imposing_radius` | 三维搜索半径 (m) |
| `static_obstacle_min_height` | 高于地面最小高度 (m) |
| `static_obstacle_max_height` | 高于地面最大高度 (m) |
| `static_obstacle_half_x` | X 检查半宽 (m) |
| `static_obstacle_half_y` | Y 检查半宽 (m) |
| `static_obstacle_min_points` | 障碍最少点数（旧 >10 即 ≥11） |
| `static_obstacle_value` | 命中后的距离赋值 (m，非实测距离) |
| `static_ground_min_neighbors` | 地面最少邻居数 |
| `radius_of_ground_connection` | 地面邻域半径 (m，亦影响连边) |

这些参数写入本次导航运行配置，不修改原模板或地图；先结束导航、修改参数、再加载地图生效。导航运行时输入锁定。搜索半径不再被 Web 后端强制覆盖为 5 m。地图体素、膨胀半径、实时避障距离仍使用已有输入。点数参数必须是整数，最小高度小于最大高度，固定赋值必须低于硬碰撞半径。

本组仅覆盖静态计数判定及其地面邻域前置条件；不涵盖上游地面分割、动态雷达层所有参数。减小检测范围、提高点数门槛或缩小高度带可能漏检，不应仅为减少红点而调整。保持机器人尺寸/碰撞门槛不变，离线检查后再进行受控验证。

## 14. 本地修改追溯：墙体误识别为地面的修复与墙根漏识别

对应本仓库提交：`1a7fa8c`，标题为“修复建图中墙体被识别成了地面点的问题”。这是此前反复回放建图、检查地面分层期间相关的代码修改；仅凭提交记录不能精确对应到某一轮对话。

主要代码位于 [imageProjection.cpp](src/dddmr_lego_loam/lego_loam_bor/src/imageProjection.cpp) 的 `ImageProjection::zPitchRollFeatureRemoval()`。这是上游地面提取与补点逻辑，不是导航静态层 `static_layer.cpp` 中的红点计数规则。后来回退静态层的“最近水平距离”算法，并未回退这里的地面提取修复。

### 14.1 修改了什么

1. **增加三维邻域法向检查。** 两个点高度接近、连线接近水平，不足以证明它们在地面上：墙面上的两个采样点也可能满足这些条件。新增逻辑在安装俯仰角修正后的原始点云中寻找邻居，计算协方差矩阵，以最小特征值对应的特征向量估计平面法向。排除近似直线、明显不共面等不可靠邻域，再要求法向接近竖直方向。水平地面的法向接近 `(0,0,1)`；竖直墙面的法向接近水平方向。
2. **上下两个候选点都要通过法向检查。** 地面标记分支和地面补点分支都增加此限制。一点落地、另一点落墙时，不再直接把这组点用于地面补片。
3. **修正左右邻点查找。** 使用同一扫描行内的列索引环绕，跳过无效/非有限点，按 XY 水平距离判断邻点间隔，不再仅比较 Y。必须左右两侧都找到有效邻居，避免只找到一侧就认为邻域有效。
4. **约束插值补点。** 插值循环中的候选补点也要通过观测支持和法向检查，要求附近 0.1 m 内有原始观测点，避免在未观测空隙中直接插值出地面。该检查使用 KD-tree 的平方距离，因此源码中的阈值 `0.01` 对应距离 `0.1 m`，不是 `0.01 m`。

当前 Astrall 建图配置中的相关参数如下（属于本地设置，不是雷达硬件规格）：

```yaml
imageProjection:
  ground_normal_check: true       # 启用新增邻域法向检查。
  ground_normal_radius: 0.20      # 在原始点云中搜索邻居的半径，单位 m。
  ground_normal_min_neighbors: 6  # 邻域至少 6 点，否则拒绝按此条件判为地面。
  ground_slope_tolerance: 0.3     # rad，约 17.2°；也用于限制法向偏离竖直方向。
```

法向条件为 `abs(normal.z) >= cos(ground_slope_tolerance)`，容差为 `0.3 rad` 时右侧约为 `0.955`。代码还通过协方差特征值比例限制邻域的平面性；不是只检查一个角度。

### 14.2 为什么修复后墙根真实地面也可能减少

墙根地面点的 0.20 m 邻域可能同时包含“水平地面点”和“竖直墙面点”。当前逻辑将它们放在一起估计一个平面，可能得到倾斜法向，或被判为不共面，于是连真实地面点也未通过检查。上下端点必须同时通过、左右邻居必须齐全的限制，也可能使边界和稀疏区域不再补点。

这是当前实现存在的**潜在漏识别机制**，并非已经证明某张地图所有墙根缺失都由它造成。需要对比同一帧的原始点云、地面输出、法向/邻域支持及补点结果，区分墙面被正确拒绝和真实地面被误拒绝。部分检查失败分支直接跳过，不能把所有未进入地面的点都说成已被识别为“墙”。

后续改进方向是将混合邻域中的墙面和地面分开拟合，利用可靠地面种子、连续性及真实观测支持恢复墙根地面；这只是改进方向，尚未在此修改中实现。不要简单关闭全部法向检查或放宽坡度阈值，否则可能重新把竖直墙面补成可通行地面。

### 14.3 哪些限制不是这次提交新增的

当前代码还包含连线坡度取绝对值、以及“坡度或高度差任一超限即拒绝”的限制。对比 `1a7fa8c` 的父提交，这些限制此前已经存在，不能全部归入本次修复。可用以下命令复查准确修改范围：

```bash
git -C /home/nvidia/sed_ros2/dddmr_navigation show 1a7fa8c -- \
  src/dddmr_lego_loam/lego_loam_bor/src/imageProjection.cpp
```

## 15. 本地实验选项：墙体投影直接补地面

Web 建图参数新增“墙体投影补地面（实验，默认关闭）”，对应 Astrall `odin1_mapping.yaml`：

```yaml
imageProjection:
  project_walls_to_ground: false
```

开始建图前勾选，运行中不可切换；恢复默认参数会取消勾选。Web 将布尔值写入本次运行 YAML，不改模板。此选项不自动延续到下一次页面打开，导航配置默认不启用现场补点。

开启后，在原地面提取完成、0.1 m 体素降采样之前，从本帧 `segmented_cloud_pure` 的非地面候选中找近似竖直平面，将符合条件的点投影到附近已有地面拟合平面，直接追加到 `patched_ground`。因此会进入 `ground_cloud`、关键帧地面及保存地面，后续加载时参与 `mapground`，不是仅供显示的独立障碍层。原始墙点、分割标记及障碍候选不删除、不改为地面标记。

当前实验实现的固定几何限制：墙邻域半径 0.20 m、地面 XY 邻域半径 0.50 m，两类邻域均至少 6 点，拒绝线状/明显不共面邻域，点到拟合平面残差不超过 0.05 m。墙法向接近水平、地面法向接近竖直（角度容差 0.3 rad），候选墙点高于投影地面 0.10～1.0 m。投影沿安装俯仰角修正后的 Z 方向进行，再变换回传感器坐标；不是统一写成 `z=0`，也不是完整的实时重力姿态校正。使用追加前的地面快照作为种子，不将本次新点反复扩展。没有附近可靠地面时不补；不会人为连接不同墙点间的空白区。

**限制：这是竖直面几何判定，不是语义墙体识别；柜体等竖直面也可能进入。种子包含原地面算法的受约束补点，并非全部为直接回波。投影点是推算地面，不证明墙下实际可行走；可能影响 MCL 地面匹配、地面图连边及避障。原墙点保留也不保证规划不会误连。** 默认关闭，不自动修补已有地图；需重新建图并另存，对比门洞、墙根、台阶及通道后再考虑导航。未经过完整 rosbag/实机验收，不应直接启用自主运动。

新增几何回归测试覆盖平地投影、无地面支持、水平面拒绝、远地面拒绝、非零地面高度及过高候选拒绝；Web 测试验证开关写入运行 YAML。日志周期性输出降采样前新增点数，方便确认开关是否产生实际补点。

## 16. 本地实验：静态红点圆柱支持校验

Astrall 导航的全局静态层增加 `static_support_check: true`，`static_support_radius: 0.10`，`static_support_min_points: 2`。通用插件默认关闭该校验；Astrall 模板按用户选择启用。当前在 YAML 配置，Web 生成运行 YAML 时保留这些字段；需重新加载导航生效，不是热更新。

静态图计算完成后，对该静态层值小于 `inscribed_radius` 的地面节点，在 XY 半径 0.1 m、高度 `[节点Z + static_obstacle_min_height, 节点Z + static_obstacle_max_height]` 的圆柱内统计 `mapcloud` 点。半径与高度边界包含在内；0/1 点撤销该层危险贡献，2 点及以上保留。三维查询球覆盖整个圆柱，不复用较小的障碍初筛半径。高度使用本次导航实际加载的配置（包括 Web 设置）。

撤销是把该静态层节点恢复到 `max_obstacle_distance`，不是删除地面节点，也不删除 PCD 或仅隐藏显示。融合仍取各层最小值，实时雷达动态危险标记不变，因此有动态支持时最终 dGraph 仍可能红色。空地图输入不作为清除依据。日志输出校验/撤销节点数。

**这会减弱静态地图的碰撞保护。节点正上方无点并不意味着机器人身体与旁边墙体无碰撞；0.2 m 地图体素也可能让 0.1 m 圆柱内真实障碍不足两点。Odin1 前向视场、遮挡、数据延迟和定位误差使动态避障不能自动构成等效保障。此项未经实机安全验收，不应凭红点减少恢复自主运动。** 可将 `static_support_check` 设为 `false` 并重新加载导航，恢复未经该校验的静态标记。

## 17. 重定位高度与平面定位约束

`initial_3d_pose` 的位置是 `robot_frame`（Astrall 为 `base_link`）在地图中的位置，不是脚底或地面点。Web 首帧选项读取 `poses.pcd` 第一条机身位姿；手动重定位优先采用最近建图轨迹的机身高度。旧 MCL 回调会再次搜索地面并覆盖该 Z，曾将传入约 -0.07 m 改成地面约 -0.51 m，导致机身及实时点云整体下沉。现已移除这一覆盖，保留传入 Z；其他客户端也必须提供机身高度。

`mcl_3dl.planar_mode` 现有实际实现（通用默认 `false`，Astrall 配置 `true`）：初始化、匹配评分前、失败扩散后以及重采样后，将粒子的 Z 固定为本次初始机身高度，roll/pitch 固定为零，继续估计 X/Y/yaw。手动重定位更新高度基准。此前只有 YAML 字段，没有对应代码，不能形成平面约束。该模式只适合平地，不用于坡道、楼梯或需要估计机身倾斜的场景；也不能修复错误外参、时间同步或航向匹配。

“使用建图第一帧”是启动先验，不是锁定位姿；后续 MCL 更新或手动重定位仍会改变位置。修复需重新加载导航进程，不必重新建图。保持运动关闭，检查日志 `planar_mode`、`base_link reference z` 和 `Set initial pose` 的 Z 一致，再同时显示静态 `mapcloud` 和实时分割云验证对齐。粒子收敛不能替代实际点云对齐验收。

新增 `planar_constraint_test` 检查高度/倾角约束、X/Y/yaw 保留和重置高度；测试与编译不代表实机对齐已通过。

<!-- 本地补充结束 -->
