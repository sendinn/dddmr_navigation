#pragma once
#include <cstdint>
#include <trajectory_generators/coupled_path_tracker.h>
#include <trajectory_generators/single_axis_tracking.h>
#include <trajectory_generators/trajectory_generator_theory.h>

namespace trajectory_generators {
/**
 * @brief 面向机械狗的连续三轴路径跟踪与朝向对齐轨迹生成器。
 *
 * 跟踪模式根据路径前视点、横向误差和朝向误差计算机体期望速度；
 * 对齐模式根据外部提供的 rotation_error_ 计算期望角速度。
 * CoupledPathTracker 负责跟踪计算、耦合补偿和速度限幅，采用线性模型：
 *   实际 vx = 指令 vx + k_xy * 指令 vy + k_xw * 指令 wz。
 * 耦合系数需要实测标定；两者为零时不进行耦合补偿。
 *
 * 直接实现 TrajectoryGeneratorTheory 接口，独立读取参数并进行轨迹积分、
 * 加减速及制动轨迹生成，使用耦合模型预测机体速度。
 * 每周期在动态速度窗口内组合采样 vx、vy、wz，并补充精确参考指令。
 * 外部 critics 检查后，从通过检查的候选中选择最接近参考指令的一条。
 * single_axis_tracking=true 时，按 yaw、Y、X 优先级选择单轴指令，切换前停稳。
 * 此模式关闭混轴前馈补偿，但仍按耦合模型预测实际位移。
 * 本类不直接发布 cmd_vel；路径阻塞时仍由上层决定停车或重新规划。
 */
class CoupledPathTrajectoryGeneratorTheory final : public TrajectoryGeneratorTheory {
 public:
  size_t getSamplingSize() override { return sample_params_.size(); }
  /// 每控制周期检查位姿/里程计有效性，并计算、限幅和保存候选指令。
  /// 输入无效时保留空候选集，交由上层处理无法生成轨迹的情况。
  void initialise() override;

  /// 按索引调用本类 generateTrajectory() 模拟候选；失败时将轨迹标为无效。
  void getSamplingTrajectoryByIndex(size_t index, base_trajectory::Trajectory& trajectory) override;

  /// 从 accepted 中选择速度指令最接近 reference_command_ 的有效轨迹。
  /// 不使用 rejected 生成新指令；没有有效候选时 best.cost_ 保持为 -1。
  void expertScoring(std::vector<base_trajectory::Trajectory>& accepted,
      std::map<std::string,std::vector<base_trajectory::Trajectory>>& rejected,
      base_trajectory::Trajectory& best) override;
 protected:
  /// 插件初始化：读取本类运动参数、跟踪参数、共享耦合系数和模式，
  /// 并检查限速、加速度及与本控制器不兼容的配置。
  void onInitialize() override;
  void configurateActuatorType() override;

  /// 将指令 [vx, vy, wz] 映射为模型预测的机体速度，供本类轨迹模拟使用。
  Eigen::Vector3f predictedBodyVelocity(const Eigen::Vector3f& command) const;
 private:
  // 独立的运动约束与模拟配置，不依赖其他生成器的数据类型或状态。
  struct Limits {
    double min_vel_x, max_vel_x, min_vel_y, max_vel_y;
    double min_vel_trans, max_vel_trans, min_vel_theta, max_vel_theta;
    double acc_lim_x, acc_lim_y, acc_lim_theta, deceleration_ratio;
    bool use_motor_constraint;
    Eigen::Vector3f getAccLimits() const {
      return Eigen::Vector3f(acc_lim_x, acc_lim_y, acc_lim_theta);
    }
  } limits_{};
  struct RolloutParams {
    double controller_frequency, sim_time, sim_granularity, angular_sim_granularity;
    int linear_x_sample = 2, linear_y_sample = 2, angular_z_sample = 10;
    pcl::PointCloud<pcl::PointXYZ> cuboid;
  } params_{};
  bool sample_braking_commands_ = true;
  double braking_reaction_time_ = 0.0;
  std::vector<Eigen::Vector3f> sample_params_;
  void sampleVelocityWindow(const Velocity& measured);
  bool generateTrajectory(Eigen::Vector3f command, base_trajectory::Trajectory& trajectory);
  Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
                                    const Eigen::Vector3f& vel, double dt);
  /// 路径几何、反馈控制及耦合模型计算，不负责 ROS 指令发布。
  CoupledPathTracker tracker_;

  /// false：连续路径跟踪；true：朝向对齐（仍执行耦合补偿）。
  bool alignment_only_ = false;

  // 可执行指令单轴化；网格候选仍保留，用于碰撞检查和诊断。
  bool single_axis_tracking_ = false;
  double axis_yaw_enter_ = 0.1745329252, axis_yaw_exit_ = 0.0872664626;
  double axis_lateral_enter_ = 0.10, axis_lateral_exit_ = 0.05;
  SingleAxisTracking axis_policy_;
  TrackingErrors axis_errors_;
  Velocity cycle_measured_{};
  int64_t cycle_stamp_ = 0, last_selection_ns_ = 0;
  uint64_t axis_epoch_ = 0;
  bool cycle_valid_ = false;


  /// 启动参数：输出逐周期诊断；序号关联参考计算与 critic 后的选择结果。
  bool tracking_diagnostics_ = false;
  uint64_t diagnostic_cycle_ = 0;

  /// 本周期经补偿和限幅后的参考指令 [vx, vy, wz]，用于生成候选及最终选择。
  Eigen::Vector3f reference_command_ = Eigen::Vector3f::Zero();
};
}
