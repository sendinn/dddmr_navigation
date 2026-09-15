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
| `scan_period` | 一帧/一圈的周期（秒） | 影响运动相关处理；与真实频率不符时，移动中更容易出现重影和轨迹旁障碍带。 |
| `vertical_angle_bottom/top` | 雷达有效垂直视场下、上边界（度） | 地面投影错行，地面可能被当作障碍，墙面也可能被误判为地面。 |
| `odom_type` | 使用外部轮式/融合里程计，或激光里程计 | 选择 `wheel_odometry` 却没有稳定 `odom` 时，地图会漂移或停更。 |
| `base_ground_frame` | 地面分割参考坐标系 | 外参不对会使地面倾斜、机器人自身点残留，并影响保存的关键帧姿态。 |

`num_horizontal_scans` 不是“实际只看见多少度”。例如水平分辨率为 `0.5°` 时，完整圆周逻辑列数通常是 `360 / 0.5 = 720`；Odin1 只有前方扇形视场时，后方未观测列应为空，不能把扇形强行拉满 360°，也不能把未观测区当成已清空区域。

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
| `full_cloud_info` | 投影后的完整点云及附加信息 | 算法内部处理与调试投影是否正确。 |
| `ground_cloud` | 被识别为地面的点 | 检查地面是否单层、坡道是否连续，以及机身/墙面是否误入。 |
| `segmented_cloud` | 分割后的有效点 | 后续特征提取和里程计匹配的主要输入。 |
| `segmented_cloud_pure` | 当前 `sendinn` 源码在地面提取阶段保留的非地面/障碍候选点 | 作为导航阶段动态障碍感知输入；当前版本并不等同于“所有有效连通分割簇”，详见 7.1 节。 |
| `outlier_cloud` | 被判为离群的点 | 诊断噪声、稀疏回波以及分割阈值。 |
| `laser_cloud_sharp` | 强边缘特征 | 用于姿态与结构约束。 |
| `laser_cloud_less_sharp` | 较宽松的边缘特征 | 用于建图匹配。 |
| `laser_cloud_flat` | 强平面特征 | 用于平面约束。 |
| `laser_cloud_less_flat` | 较宽松的平面特征 | 用于建图和地图表达。 |
| `laser_odom_to_init` | 激光里程计结果 | 表示建图过程中的相对运动估计，不等同于导航阶段最终 MCL 定位。 |

地面点正常情况下可以有坡度和厚度，但同一处静止地面不应长期出现明显的多层平行结构。多层地面常见原因是时间戳不同步、动态 TF/外参错误、扫描周期不对、拼帧过多或机械狗运动时机身俯仰未被正确补偿。

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

<!-- 本地补充结束 -->
