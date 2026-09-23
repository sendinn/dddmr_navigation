# Astrall continuous coupled path controller

`trajectory_generators::CoupledPathTrajectoryGeneratorTheory` implements the design in
`robot_path_tracking_with_coupling.md` as a DDDMR trajectory generator plugin.
It computes a nearest path projection, signed lateral error (left positive),
arc-length lookahead and heading feedback. Path-tangent progression and normal
correction are transformed into the body frame. Forward progress slows with
heading/lateral error and near the reference endpoint. All three command axes
can be nonzero. Small continuous deadbands suppress noise.

The first model is `vx_actual = vx_cmd + k_xy * vy_cmd + k_xw * wz_cmd`.
Compensation and collision rollout use the same coefficients. Uniform command
scaling respects asymmetric axis bounds and both commanded and predicted
translation limits without destroying the compensation relation. Bounds must
include zero. The model covers steady coupling; actual response delay,
acceleration, stopping and speed-dependent/asymmetric drift still need measurement.

## Integration

The controller directly implements `TrajectoryGeneratorTheory` and lives alongside
other plugins in DDDMR's `trajectory_generators` package. It owns its motion limits,
parameter loading, candidate list and coupling-aware rollout. It does not inherit
from or include the Omni generator or its parameter/limit types. Only the generic
`braking_rollout.h` velocity integration helper is shared. Astrall configuration
stays in `astrall_dddmr_bringup`. Measured initial velocity, finite acceleration,
reaction time, braking tail and oriented cuboid generation are preserved.

Each cycle samples the command-space dynamic velocity window using
`linear_x_sample`, `linear_y_sample`, and `angular_z_sample` (defaults 2, 2, 10).
The original `VelocityIterator` adds zero for intervals crossing zero and
collapses equal bounds to one value. Translational deceleration windows and
zero-inclusive braking sampling are retained. The exact compensated reference
command is also added unless already present. Thus counts are not always the
product of the configured counts (a symmetric 2/2/10 window has 99 grid entries,
plus at most one reference command). This applies to both plugin instances.
All candidates use coupling-aware prediction and respect both command and
predicted translational speed limits before critic checks. The plugin still
selects the accepted command closest to the reference, rather than switching
to Omni's scoring policy. Multiple directions are now available; upstream path
blockage handling can still stop/replan before local selection. A rejected
command is never published or modified after scoring. Invalid/stale pose or odometry and invalid paths produce no
candidates, triggering the existing failure/stop handling.

`coupled_path_tracking` is the normal tracking instance, selected by
`p2p_move_base.main_trajectory_generator` and matched by
`local_planner.tracking_trajectory_generator` for blockage checks. Critics
reference this same instance name. `coupled_heading_alignment` is selected by
`p2p_move_base.heading_trajectory_generator` and uses a second instance with
`alignment_only: true`, ensuring initial/final rotation prediction includes X
compensation too. Heading error is supplied by the existing task state machine.
Both instances read the same `trajectory_generators` node parameters
`astrall_coupling_xy` and `astrall_coupling_xw`.

These are two instances of `trajectory_generators::CoupledPathTrajectoryGeneratorTheory`, not
separate theory classes. The disabled rotate recovery still uses the actual
`DDRotateInplaceTheory` instance `differential_drive_rotate_inplace`.
Legacy instance names remain parameter defaults for other robot configurations;
Astrall explicitly selects the coupled instances. Continuous mode rejects a
mismatch between the task's main instance and the local planner's tracking instance.

`continuous_path_tracking: true` on `p2p_move_base` bypasses standalone rotation
pulse admission only during `d_controlling`. Initial/final alignment still uses
angle feedback, timeout and stopping confirmation. Reaching the XY goal also
starts a three-fresh-sample stop check before reporting success, even if the
final heading is already within tolerance. Use this mode with the new
plugin; do not enable legacy fixed-duration rotation prediction with it.

## Build and select

From `/home/nvidia/sed_ros2/dddmr_navigation`:

```bash
source setup_dddmr.bash
colcon build --packages-select trajectory_generators local_planner p2p_move_base --parallel-workers 1
source install/setup.bash
colcon test --packages-select trajectory_generators p2p_move_base
```

From `/home/nvidia/sed_ros2/robot/astrall`:

```bash
source /home/nvidia/sed_ros2/dddmr_navigation/setup_dddmr.bash
colcon build --packages-select astrall_dddmr_bringup --parallel-workers 1
source install/setup.bash
```

The existing navigation launch uses this configuration by default:

```text
/home/nvidia/sed_ros2/robot/astrall/astrall_dddmr_bringup/config/odin1_navigation.yaml
```

This is the single Astrall navigation configuration, shared by the launch file
and WebUI. The two controller instances retain existing body dimensions and
empirical braking limits. The
profile bounds each command axis to 0.1 (m/s or rad/s), matching the default
Astrall driver clamps. Cruise speed is 0.08 m/s. Initial/final alignment timeout
is 60 seconds for this low angular speed. If driver clamps are changed, keep
controller bounds within them; downstream per-axis clipping destroys compensation.
The control rate stays at 10 Hz to match the current task/local-planner loops;
20–50 Hz requires measuring CPU time and updating all three frequency settings.

Use `setup_dddmr.bash` for the custom PCL 1.15 prefix. Mixing system PCL 1.12
with this DDDMR installation is ABI-incompatible. If CMake has already cached
system PCL, rebuild this package with `--cmake-clean-cache` after sourcing it.

## Calibration and diagnostics

Coupling coefficients default to **zero, uncalibrated**, not the example numbers
in the design note. `k_xy` is dimensionless; `k_xw` is m/rad. Compare time-aligned
commands and base-frame odometry after verifying lidar extrinsics. Fit positive
and negative directions and check repeatability before enabling compensation.
A yaw or Y command can require negative X compensation even while overall
motion stays forward. Nonlinear or same-sign drift in both turn directions
requires a richer model; these two linear coefficients cannot describe it.

The periodic plugin log includes lateral/heading error, lookahead, desired body
velocity and command. DDDMR retains the generated/accepted/selected trajectory
visualizations. Prediction is not a measurement of physical execution.

Offline tests cover signs, coordinate transforms, lookahead, endpoints,
saturation, compensation and closed-loop convergence in the assumed model.
The plugin test also loads the installed class with a self-contained YAML fixture and invokes
the real DDDMR collision critic on a braking trajectory.
They do not establish true robot gains, mixed-axis SDK response, or real stopping
distance. Live validation should measure straight tracking, both lateral-error
signs, bends, initial/final alignment and obstacle braking before raising speed.

The plugin is registered in `trajectory_generators.xml` and compiled into
`libtheories.so`. There is no separate `astrall_path_controller` ROS package.

## 横移振荡诊断

`coupled_path_tracking.tracking_diagnostics: true` 启用逐周期日志，默认关闭；
修改后重启节点生效。当前 Astrall 配置为排查振荡开启此项，诊断后可关闭以减少日志。
`tracking_diag` 的实例名与 `cycle` 共同关联同一周期：

- `phase=reference`：横向/朝向误差、机体朝向、最近路径段方向、前视参考方向、
  原始期望速度、加速度限制后速度、补偿限幅后指令、实测机体速度及数据年龄。
- `phase=selection`：critic 后选中的候选或无有效候选；它不是最终发布或 SDK 执行确认。
  输入无效提前返回时没有 reference 日志；未进入 expertScoring 时没有 selection 日志。
- 速度顺序均为 `(vx, vy, wz)`，平移单位 m/s，角速度 rad/s，方向角单位 rad。

当前仅跟踪实例将横向死区调为 0.05 m、横向增益调为 0.3 /s、最大法向纠偏速度
调为 0.03 m/s。这是降低纠偏强度的待验证初值，可能增加跟踪偏差和回归时间，
不代表已确定振荡根因或完成真机验证。

## 单轴执行模式

Astrall 当前两个实例均启用 `single_axis_tracking: true`：朝向偏差超过
`axis_yaw_enter` 优先纯 yaw，回到 `axis_yaw_exit` 以内退出；横向偏差超过
`axis_lateral_enter` 使用纯 Y，回到 `axis_lateral_exit` 内退出；否则使用纯 X。
对齐实例始终只请求 yaw。切轴或反向先选择经过检查的零指令，连续三个新的
里程计样本确认平移各轴 ≤0.03 m/s、角速度 ≤0.05 rad/s 后准入新轴。
新任务/重规划参考 epoch 改变、过期数据、时间倒退或选择间隔超过 0.5 s 重置状态。
状态机只在当前被执行实例的 expertScoring 中更新，避免所有插件同时初始化时提前放行。

网格仍显示混轴候选，但最终执行必须是准入轴、准入方向且不超过该轴参考速度。
精确补充候选也改为各轴独立指令。未通过 critics 的指令不能补发，检查后不再清零
或改写其他分量。单轴模式不使用混轴前馈补偿；耦合系数仍用于预测纯指令实际造成
的位移，所以纯 yaw/Y 指令并不保证机体完全没有 X 漂移。
`reference_command` 日志在此模式表示各轴独立目标，实际执行参考见 admission/selection。
将 `single_axis_tracking` 设为 false 可恢复连续三轴模式。

## 先转向、后直行候选（当前行走配置）

`coupled_path_tracking.turn_then_forward: true` 覆盖本实例的三轴速度网格及
X/Y/yaw 优先级选择，改成方向角与正 X 速度的组合。`angular_z_sample` 为方向数量，
`turn_angle_range` 为相对当前机体朝向的采样半范围；另加入当前朝向和前视目标方向。
只保留面向前视目标半平面的方向。`linear_x_sample` 为正前进速度档数，
最大值由巡航、轴限速、平移限速和接近目标减速共同决定；Y 采样在此模式不使用。

每个候选预测反应延迟、纯 yaw 转向、停车延迟与制动、纯 X 直线段及最终制动。
转向漂移来自现有耦合模型，不伪造原地旋转；直行段不含持续 Y/yaw 指令。
所有阶段的车身包络都经过 critics。按整条轨迹代价、终点到前视点距离及
方向误差选择，停车只作无有效运动候选或阶段门控时的退路。

实际仅发布所选候选的当前阶段指令：先停稳三个新样本，选择并锁定世界坐标系方向，
旋转进入 `axis_yaw_exit`，停车并确认三个新样本，再沿 X 直行。行走中不混合转向。
走完一个前视距离、路径方向变化超过 `axis_yaw_enter` 或机体偏离锁定方向超过
`axis_yaw_exit` 时先停车重新选择。新任务、参考 epoch 更新、时钟倒退或
超过 0.5 秒未执行本实例时重新确认停稳。`turn_timeout` 限制单次转向及其预测时间。

诊断 `phase=turn_forward` 的 stage：0 停车、1 方向选择、2 转向、3 停稳、4 直行。
`selected_valid` 为生成器选择有效性，不是实际执行确认。初始/终点朝向对齐实例
继续独立工作，在单轴模式仅生成纯 yaw 和停车候选。局部前方路径阻塞检查仍可
要求停车重规划；新的候选形式不绕过该检查。

### 旋转测试入口

WebUI 的 `/api/rotation-test` 调用任务节点 `/rotation_test` Action（复用
`PToPMoveBase` 类型，`target_value` 为相对角度弧度）。任务节点与导航互斥，
将世界坐标目标朝向通过 `rotation_test_ / rotation_test_heading_` 传给主生成器。
测试复用 `prepareTurnCandidates / generateTurnTrajectory / selectTurnTrajectory`：
只保留指定朝向及停车，旋转候选速度为零前进速度，预测到旋转制动完成即结束。
达到容差并确认停稳后设置 `rotation_test_complete_`，不会执行 Drive 的前进指令。

局部规划器为评分建立短参考段，保留感知检查和全部配置 critics；测试不执行前向
参考路径扫描或全局重规划，因为该任务没有前进段。旋转包络被评分器拒绝时结束测试并
记录评分器名称，而不是改选另一个转向目标。独立测试不代表正常导航的前向路径已可通行。
