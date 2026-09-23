/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <trajectory_generators/coupled_path_trajectory_generator_theory.h>
#include <dddmr_sys_core/motion_timestamp.h>
#include <pluginlib/class_list_macros.hpp>
#include <trajectory_generators/braking_rollout.h>
#include <trajectory_generators/velocity_iterator.h>
#include <pcl/common/common.h>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::CoupledPathTrajectoryGeneratorTheory,
                      trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators {
void CoupledPathTrajectoryGeneratorTheory::onInitialize() {
  TrackerConfig c;
  auto parameter=[&](const char* key,double value) {
    return node_->declare_parameter<double>(name_+"."+key,value);
  };
  limits_.min_vel_x=parameter("min_vel_x",-0.1);
  limits_.max_vel_x=parameter("max_vel_x",0.1);
  limits_.min_vel_y=parameter("min_vel_y",-0.1);
  limits_.max_vel_y=parameter("max_vel_y",0.1);
  limits_.min_vel_trans=parameter("min_vel_trans",0.0);
  limits_.max_vel_trans=parameter("max_vel_trans",0.1);
  limits_.min_vel_theta=parameter("min_vel_theta",0.0);
  limits_.max_vel_theta=parameter("max_vel_theta",0.1);
  limits_.acc_lim_x=parameter("acc_lim_x",0.3);
  limits_.acc_lim_y=parameter("acc_lim_y",0.3);
  limits_.acc_lim_theta=parameter("acc_lim_theta",0.5);
  limits_.deceleration_ratio=parameter("deceleration_ratio",2.0);
  params_.controller_frequency=parameter("controller_frequency",10.0);
  params_.sim_time=parameter("sim_time",2.0);
  params_.sim_granularity=parameter("sim_granularity",0.1);
  params_.angular_sim_granularity=parameter("angular_sim_granularity",0.05);
  // 兼容原 Omni YAML 的浮点表示，数量必须是有限正整数。
  auto samples=[&](const char* key,double fallback) {
    const double value=parameter(key,fallback);
    if (!std::isfinite(value) || value<1 || value>100 || std::floor(value)!=value)
      throw std::invalid_argument(std::string(key)+" must be an integer in [1,100]");
    return static_cast<int>(value);
  };
  params_.linear_x_sample=samples("linear_x_sample",2.0);
  params_.linear_y_sample=samples("linear_y_sample",2.0);
  params_.angular_z_sample=samples("angular_z_sample",10.0);
  // 兼容现有 YAML，但不引入单轴或轮式运动策略。
  single_axis_tracking_ = node_->declare_parameter<bool>(name_+".single_axis_tracking",false);
  const bool axis_obstacle_avoidance = node_->declare_parameter<bool>(name_+".axis_obstacle_avoidance",false);
  limits_.use_motor_constraint=node_->declare_parameter<bool>(name_+".use_motor_constraint",false);
  sample_braking_commands_=node_->declare_parameter<bool>(name_+".sample_braking_commands",true);
  braking_reaction_time_=parameter("braking_reaction_time",0.0);
  if (!std::isfinite(braking_reaction_time_) || braking_reaction_time_<0 || braking_reaction_time_>2)
    throw std::invalid_argument("braking_reaction_time must be in [0,2] seconds");
  // 碰撞检测依赖此前四个顶点的顺序：blb、brb、blt、flb。
  params_.cuboid.clear();
  for (const char* corner : {"blb","brb","blt","flb","brt","frt","flt","frb"}) {
    const std::string key=name_+".cuboid."+corner;
    node_->declare_parameter(key,rclcpp::PARAMETER_DOUBLE_ARRAY);
    const auto point=node_->get_parameter(key).as_double_array();
    if (point.size()!=3 || !finite({point[0],point[1],point[2]}))
      throw std::invalid_argument(key+" must contain three finite coordinates");
    params_.cuboid.push_back(pcl::PointXYZ(point[0],point[1],point[2]));
  }
  c.lookahead=parameter("lookahead_distance",c.lookahead);
  c.lateral_gain=parameter("lateral_gain",c.lateral_gain);
  c.yaw_gain=parameter("yaw_gain",c.yaw_gain);
  c.lateral_deadband=parameter("lateral_deadband",c.lateral_deadband);
  c.yaw_deadband=parameter("yaw_deadband",c.yaw_deadband);
  c.lateral_slow_distance=parameter("lateral_slow_distance",c.lateral_slow_distance);
  c.approach_gain=parameter("approach_gain",c.approach_gain);
  c.cruise_speed=parameter("cruise_speed",c.cruise_speed);
  c.max_lateral_correction=parameter("max_lateral_correction",c.max_lateral_correction);
  // Both tracking and alignment instances use the same measured robot model.
  auto robot_parameter=[&](const char* key) {
    if (!node_->has_parameter(key)) node_->declare_parameter<double>(key,0.0);
    return node_->get_parameter(key).as_double();
  };
  c.k_xy=robot_parameter("astrall_coupling_xy");
  c.k_xw=robot_parameter("astrall_coupling_xw");
  tracker_=CoupledPathTracker(c);
  alignment_only_=node_->declare_parameter<bool>(name_+".alignment_only",false);
  turn_then_forward_=node_->declare_parameter<bool>(name_+".turn_then_forward",false);
  turn_angle_range_=parameter("turn_angle_range",1.5707963268);
  turn_timeout_=parameter("turn_timeout",15.0);
  if (!std::isfinite(turn_angle_range_) || turn_angle_range_<=0 || turn_angle_range_>std::acos(-1.0) ||
      !std::isfinite(turn_timeout_) || turn_timeout_<1 || turn_timeout_>60 ||
      (turn_then_forward_ && (!single_axis_tracking_ || alignment_only_)))
    throw std::invalid_argument("turn_then_forward requires single-axis tracking (not alignment), angle in (0,pi], timeout in [1,60]");
  tracking_diagnostics_=node_->declare_parameter<bool>(name_+".tracking_diagnostics",false);
  axis_yaw_enter_=parameter("axis_yaw_enter",0.1745329252);
  axis_yaw_exit_=parameter("axis_yaw_exit",0.0872664626);
  axis_lateral_enter_=parameter("axis_lateral_enter",0.10);
  axis_lateral_exit_=parameter("axis_lateral_exit",0.05);
  if (!std::isfinite(axis_yaw_enter_) || !std::isfinite(axis_yaw_exit_) ||
      !std::isfinite(axis_lateral_enter_) || !std::isfinite(axis_lateral_exit_) ||
      axis_yaw_exit_<=0 || axis_yaw_enter_<=axis_yaw_exit_ || axis_yaw_enter_>=1.5707963268 ||
      axis_lateral_exit_<c.lateral_deadband || axis_lateral_enter_<=axis_lateral_exit_ ||
      axis_yaw_exit_<=c.yaw_deadband)
    throw std::invalid_argument("Invalid single-axis enter/exit thresholds or deadbands");
  if (axis_obstacle_avoidance || !sample_braking_commands_ ||
      limits_.min_vel_trans!=0 || limits_.min_vel_theta!=0 || limits_.use_motor_constraint)
    throw std::invalid_argument("CoupledPathTrajectoryGeneratorTheory requires axis_obstacle_avoidance=false, sample_braking_commands=true, min_vel_trans=0, min_vel_theta=0, use_motor_constraint=false");
  for (double v : {limits_.min_vel_x,limits_.max_vel_x,limits_.min_vel_y,limits_.max_vel_y,
                   limits_.max_vel_theta,limits_.max_vel_trans,limits_.acc_lim_x,limits_.acc_lim_y,
                   limits_.acc_lim_theta,limits_.deceleration_ratio,params_.controller_frequency,
                   params_.sim_time,params_.sim_granularity,params_.angular_sim_granularity})
    if (!std::isfinite(v)) throw std::invalid_argument("Non-finite controller limit");
  if (limits_.min_vel_x>0 || limits_.max_vel_x<=0 || limits_.min_vel_y>0 || limits_.max_vel_y<0 ||
      limits_.max_vel_theta<=0 || limits_.max_vel_trans<=0 || limits_.acc_lim_x<=0 ||
      limits_.acc_lim_y<=0 || limits_.acc_lim_theta<=0 || limits_.deceleration_ratio<=0 ||
      params_.controller_frequency<=0 || params_.sim_time<=0 || params_.sim_time>10 ||
      params_.sim_granularity<=0 || params_.angular_sim_granularity<=0)
    throw std::invalid_argument("Invalid controller bounds, acceleration, or rollout horizon");
  for (const auto& p:params_.cuboid)
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
      throw std::invalid_argument("Non-finite robot cuboid");
  RCLCPP_INFO(node_->get_logger(), "%s: Astrall %s %s, coupling_xy=%.4f, coupling_xw=%.4f (requires measured calibration)",
              name_.c_str(),turn_then_forward_?"turn_then_forward":single_axis_tracking_?"single_axis":"continuous",
              alignment_only_?"alignment":"tracking",c.k_xy,c.k_xw);
}

Eigen::Vector3f CoupledPathTrajectoryGeneratorTheory::predictedBodyVelocity(const Eigen::Vector3f& command) const {
  const auto v=tracker_.predict({command[0],command[1],command[2]});
  return Eigen::Vector3f(v[0],v[1],v[2]);
}

void CoupledPathTrajectoryGeneratorTheory::initialise() {
  ++diagnostic_cycle_;
  sample_params_.clear();
  turn_candidates_.clear();
  turn_generated_.clear();
  reference_command_.setZero();
  cycle_valid_=false;
  axis_errors_=TrackingErrors{};
  const auto& odom=shared_data_->robot_state_;
  const auto& pose=shared_data_->robot_pose_;
  const auto now=node_->now();
  const rclcpp::Time odom_stamp(odom.header.stamp), pose_stamp(pose.header.stamp);
  const auto& v=odom.twist.twist;
  if (!dddmr_sys_core::motionTimestampFresh(odom_stamp.nanoseconds(),(now-odom_stamp).seconds()) ||
      !dddmr_sys_core::motionTimestampFresh(pose_stamp.nanoseconds(),(now-pose_stamp).seconds()) ||
      !finite({v.linear.x,v.linear.y,v.angular.z})) return;
  const auto& p=pose.transform.translation;
  const auto& r=pose.transform.rotation;
  tf2::Quaternion q(r.x,r.y,r.z,r.w);
  if (!finite({p.x,p.y,p.z}) || !std::isfinite(q.length2()) || std::abs(q.length2()-1.0)>1e-3) return;
  double roll,pitch,yaw;
  tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
  Velocity desired{};
  Reference reference;
  if (alignment_only_) {
    if (!std::isfinite(shared_data_->rotation_error_)) return;
    desired=tracker_.alignment(shared_data_->rotation_error_);
    axis_errors_.valid=true;
    axis_errors_.heading=wrap(shared_data_->rotation_error_);
  } else {
    std::vector<Point> path;
    for (const auto& point:shared_data_->prune_plan_.poses)
      path.push_back({point.pose.position.x,point.pose.position.y});
    reference=tracker_.reference(path,p.x,p.y,yaw);
    if (!reference.valid) return;
    desired=reference.desired;
    if (single_axis_tracking_ && !turn_then_forward_) {
      // 使用同一前方参考线计算朝向与横向偏差，避免短路径段方向不一致。
      axis_errors_=trackingErrors(path,p.x,p.y,yaw,tracker_.config().lookahead);
      if (!axis_errors_.valid) return;
      const auto& c=tracker_.config();
      desired={std::copysign(std::min(c.cruise_speed,c.approach_gain*std::abs(axis_errors_.forward)),axis_errors_.forward),
        std::clamp(c.lateral_gain*deadband(axis_errors_.lateral,c.lateral_deadband),
                   -c.max_lateral_correction,c.max_lateral_correction),
        c.yaw_gain*deadband(axis_errors_.heading,c.yaw_deadband)};
      reference.lateral=-axis_errors_.lateral; // 日志沿用机器人在路径左侧为正。
      reference.heading=axis_errors_.heading;
      reference.projection=axis_errors_.reference_start;
      reference.lookahead=axis_errors_.reference_end;
      reference.tangent_heading=wrap(yaw+axis_errors_.heading);
    }
  }
  // No previous-command state: new goals, stops and time resets cannot retain
  // an old steering target. Bound changes from measured body velocity instead.
  const Velocity raw_desired=desired;
  const Velocity measured{v.linear.x,v.linear.y,v.angular.z};
  const Velocity acc{limits_.acc_lim_x,limits_.acc_lim_y,limits_.acc_lim_theta};
  if (!single_axis_tracking_ && !alignment_only_ && std::hypot(desired[0],desired[1])+std::abs(desired[2])>1e-8) {
    for (size_t i=0;i<3;++i) {
      const double rate=acc[i]*(std::abs(desired[i])<std::abs(measured[i])?limits_.deceleration_ratio:1.0);
      desired[i]=std::clamp(desired[i],measured[i]-rate/params_.controller_frequency,
                                     measured[i]+rate/params_.controller_frequency);
    }
  }
  double translation_limit=limits_.max_vel_trans;
  const double zone_limit=shared_data_->current_allowed_max_linear_speed_;
  if (!std::isfinite(zone_limit)) return;
  if (zone_limit>0) translation_limit=std::min(translation_limit,zone_limit);
  auto command=tracker_.boundedCommand(desired,
    {limits_.min_vel_x,limits_.min_vel_y,-limits_.max_vel_theta},
    {limits_.max_vel_x,limits_.max_vel_y,limits_.max_vel_theta},translation_limit);
  if (single_axis_tracking_) {
    // 三个值表示各轴独立的目标速度，绝不作为混合指令执行。
    // 逐轴限幅避免禁用 Y 时将 X/yaw 一并缩零；不添加其他轴的前馈补偿。
    const Velocity lower{limits_.min_vel_x,limits_.min_vel_y,-limits_.max_vel_theta};
    const Velocity upper{limits_.max_vel_x,limits_.max_vel_y,limits_.max_vel_theta};
    for (int axis=0;axis<3;++axis) {
      command[axis]=std::clamp(desired[axis],lower[axis],upper[axis]);
      Velocity pure{}; pure[axis]=command[axis];
      const auto predicted=tracker_.predict(pure);
      const double translation=std::max(std::hypot(pure[0],pure[1]),std::hypot(predicted[0],predicted[1]));
      if (translation>translation_limit) command[axis]*=translation_limit/translation;
    }
  }
  cycle_measured_=measured;
  cycle_stamp_=odom_stamp.nanoseconds();
  cycle_valid_=true;
  reference_command_=Eigen::Vector3f(command[0],command[1],command[2]);
  if (turn_then_forward_ && !alignment_only_) prepareTurnCandidates(reference,p.x,p.y,yaw);
  else sampleVelocityWindow(measured);
  if (turn_then_forward_ && !alignment_only_) {
    if (tracking_diagnostics_) RCLCPP_INFO(node_->get_logger(),
      "%s tracking_diag cycle=%llu phase=turn_reference stage=%d target=(%.4f,%.4f) "
      "desired_heading=%.4f robot_yaw=%.4f measured=(%.4f,%.4f,%.4f) candidates=%zu",
      name_.c_str(),static_cast<unsigned long long>(diagnostic_cycle_),static_cast<int>(turn_phase_),
      turn_goal_[0],turn_goal_[1],turn_desired_heading_,yaw,measured[0],measured[1],measured[2],getSamplingSize());
    return;
  }
  // 此处记录参考指令，实际通过 critics 的选择结果在 expertScoring 中记录。
  if (tracking_diagnostics_) {
    const double yaw_error=alignment_only_?shared_data_->rotation_error_:reference.heading;
    RCLCPP_INFO(node_->get_logger(),
      "%s tracking_diag cycle=%llu phase=reference mode=%s lateral_error=%.4f yaw_error=%.4f "
      "robot_yaw=%.4f tangent_heading=%.4f reference_heading=%.4f "
      "projection=(%.4f,%.4f) lookahead=(%.4f,%.4f) "
      "raw_desired=(%.4f,%.4f,%.4f) limited_desired=(%.4f,%.4f,%.4f) "
      "reference_command=(%.4f,%.4f,%.4f) measured=(%.4f,%.4f,%.4f) "
      "odom_age=%.4f pose_age=%.4f candidates=%zu",
      name_.c_str(),static_cast<unsigned long long>(diagnostic_cycle_),
      alignment_only_?"alignment":"tracking",reference.lateral,yaw_error,yaw,
      alignment_only_?std::numeric_limits<double>::quiet_NaN():reference.tangent_heading,
      wrap(yaw+yaw_error),reference.projection[0],reference.projection[1],
      reference.lookahead[0],reference.lookahead[1],
      raw_desired[0],raw_desired[1],raw_desired[2],desired[0],desired[1],desired[2],
      command[0],command[1],command[2],measured[0],measured[1],measured[2],
      (now-odom_stamp).seconds(),(now-pose_stamp).seconds(),getSamplingSize());
  }
  RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
    "%s: lateral_error=%.3f, yaw_error=%.3f, lookahead=(%.3f,%.3f), desired=(%.3f,%.3f,%.3f), command=(%.3f,%.3f,%.3f)",
    name_.c_str(),reference.lateral,alignment_only_?shared_data_->rotation_error_:reference.heading,
    reference.lookahead[0],reference.lookahead[1],desired[0],desired[1],desired[2],command[0],command[1],command[2]);
}

// 与原速度网格相同：动态窗口、平移减速窗口、补零、三轴笛卡尔积。
// 采样的是下发指令；真实机体运动由后续耦合模型和制动模拟预测。
void CoupledPathTrajectoryGeneratorTheory::sampleVelocityWindow(const Velocity& measured) {
  const Eigen::Vector3f lower(limits_.min_vel_x,limits_.min_vel_y,-limits_.max_vel_theta);
  const Eigen::Vector3f upper(limits_.max_vel_x,limits_.max_vel_y,limits_.max_vel_theta);
  const Eigen::Vector3f acc=limits_.getAccLimits();
  Eigen::Vector3f lo,hi;
  for (int axis=0;axis<3;++axis) {
    hi[axis]=std::min(static_cast<double>(upper[axis]),measured[axis]+acc[axis]/params_.controller_frequency);
    lo[axis]=std::max(static_cast<double>(lower[axis]),measured[axis]-acc[axis]/params_.controller_frequency);
    if (axis<2) {
      if (measured[axis]>=upper[axis]/limits_.deceleration_ratio)
        lo[axis]=std::max(static_cast<double>(lower[axis]),measured[axis]/limits_.deceleration_ratio);
      else if (measured[axis]<=lower[axis]/limits_.deceleration_ratio)
        hi[axis]=std::min(static_cast<double>(upper[axis]),measured[axis]/limits_.deceleration_ratio);
    }
    lo[axis]=std::clamp(lo[axis],lower[axis],upper[axis]);
    hi[axis]=std::clamp(hi[axis],lower[axis],upper[axis]);
    // 即使当前速度超过限制，也允许零指令，模拟其真实制动过程。
    lo[axis]=std::min(lo[axis],0.0f);
    hi[axis]=std::max(hi[axis],0.0f);
  }
  VelocityIterator x(lo[0],hi[0],params_.linear_x_sample);
  VelocityIterator y(lo[1],hi[1],params_.linear_y_sample);
  VelocityIterator w(lo[2],hi[2],params_.angular_z_sample);
  for (;!x.isFinished();x++) {
    for (;!y.isFinished();y++) {
      for (;!w.isFinished();w++) {
        if (alignment_only_ && single_axis_tracking_ &&
            (std::abs(x.getVelocity())>1e-6 || std::abs(y.getVelocity())>1e-6)) continue;
        sample_params_.emplace_back(x.getVelocity(),y.getVelocity(),w.getVelocity());
      }
      w.reset();
    }
    y.reset();
  }
  auto append_unique=[&](const Eigen::Vector3f& command) {
    const bool found=std::any_of(sample_params_.begin(),sample_params_.end(),
      [&](const Eigen::Vector3f& candidate) {
        return (candidate-command).cwiseAbs().maxCoeff()<1e-6f;
      });
    if (!found) sample_params_.push_back(command);
  };
  if (single_axis_tracking_) {
    for (int axis=0;axis<3;++axis) {
      Eigen::Vector3f pure=Eigen::Vector3f::Zero();
      pure[axis]=reference_command_[axis];
      append_unique(pure);
    }
  } else append_unique(reference_command_);

}

void CoupledPathTrajectoryGeneratorTheory::getSamplingTrajectoryByIndex(size_t index,base_trajectory::Trajectory& trajectory) {
  trajectory.cost_=-1;
  trajectory.resetPoses();
  if (turn_then_forward_ && !alignment_only_) {
    if (index>=turn_candidates_.size() || !generateTurnTrajectory(turn_candidates_[index],trajectory)) {
      trajectory.resetPoses(); trajectory.cost_=-1;
    } else turn_generated_[turnKey(trajectory)]=turn_candidates_[index];
    return;
  }
  if (index>=sample_params_.size() || !generateTrajectory(sample_params_[index],trajectory)) {
    trajectory.resetPoses();
    trajectory.cost_=-1;
  }
}

void CoupledPathTrajectoryGeneratorTheory::expertScoring(std::vector<base_trajectory::Trajectory>& accepted,
    std::map<std::string,std::vector<base_trajectory::Trajectory>>&,
    base_trajectory::Trajectory& best) {
  best.cost_=-1;
  if (turn_then_forward_ && !alignment_only_) {
    selectTurnTrajectory(accepted,best);
    return;
  }
  int admitted_axis=-1;
  Eigen::Vector3f selection_reference=reference_command_;
  if (single_axis_tracking_) {
    const int64_t now=node_->now().nanoseconds();
    const bool fresh=cycle_valid_ && dddmr_sys_core::motionTimestampFresh(
      cycle_stamp_,static_cast<double>(now-cycle_stamp_)*1e-9);
    // 只在被选中实例的 expertScoring 更新状态；其他实例初始化不能预先累积停稳样本。
    if (axis_epoch_!=shared_data_->rotation_reference_epoch_ || last_selection_ns_<=0 ||
        now<last_selection_ns_ || now-last_selection_ns_>500000000 || !fresh)
      axis_policy_=SingleAxisTracking{};
    axis_epoch_=shared_data_->rotation_reference_epoch_;
    last_selection_ns_=now;
    admitted_axis=axis_policy_.choose(axis_errors_,cycle_measured_,cycle_stamp_,fresh,
      axis_yaw_enter_,axis_yaw_exit_,axis_lateral_enter_,axis_lateral_exit_,true,
      alignment_only_?2:-1,axis_errors_.heading>=0?1:-1);
    selection_reference.setZero();
    if (admitted_axis>=0) selection_reference[admitted_axis]=reference_command_[admitted_axis];
    if (tracking_diagnostics_) RCLCPP_INFO(node_->get_logger(),
      "%s tracking_diag cycle=%llu phase=admission axis=%d sign=%d reason=%s",
      name_.c_str(),static_cast<unsigned long long>(diagnostic_cycle_),admitted_axis,
      axis_policy_.sign(),admitted_axis<0?axis_policy_.stopReason():"single_axis_allowed");
  }
  double closest=std::numeric_limits<double>::infinity();
  // Never synthesize a command after collision scoring. Prefer the accepted
  // command closest to the continuous controller's request; zero is fallback.
  for (const auto& t:accepted) {
    if (!std::isfinite(t.cost_) || t.cost_<0) continue;
    const Eigen::Vector3f command(t.xv_,t.yv_,t.thetav_);
    if (!command.allFinite()) continue;
    if (single_axis_tracking_ && !singleAxisCommandAllowed(t.xv_,t.yv_,t.thetav_,
        admitted_axis,axis_policy_.sign(),admitted_axis<0?0.0:std::abs(selection_reference[admitted_axis]))) continue;
    const double distance=(command-selection_reference).squaredNorm();
    if (distance<closest) {closest=distance; best=t;}
  }
  if (tracking_diagnostics_) {
    // selected 是生成器选择结果；上层停车门控仍可能阻止发布，不能视为已执行。
    if (best.cost_>=0) {
      RCLCPP_INFO(node_->get_logger(),
        "%s tracking_diag cycle=%llu phase=selection status=selected accepted=%zu "
        "selected=(%.4f,%.4f,%.4f) cost=%.4f reference_distance_sq=%.6f",
        name_.c_str(),static_cast<unsigned long long>(diagnostic_cycle_),accepted.size(),
        static_cast<double>(best.xv_),static_cast<double>(best.yv_),
        static_cast<double>(best.thetav_),static_cast<double>(best.cost_),closest);
    } else {
      RCLCPP_INFO(node_->get_logger(),
        "%s tracking_diag cycle=%llu phase=selection status=no_valid_candidate accepted=%zu",
        name_.c_str(),static_cast<unsigned long long>(diagnostic_cycle_),accepted.size());
    }
  }
}
void CoupledPathTrajectoryGeneratorTheory::configurateActuatorType() {
  actuator_type_=dddmr_sys_core::ActuatorType::MOTOR;
}

// 从实测速度开始，依次预测反应延迟、指令执行及完整制动尾段。
bool CoupledPathTrajectoryGeneratorTheory::generateTrajectory(
      Eigen::Vector3f sample_target_vel,
      base_trajectory::Trajectory& traj) {

  traj.actuator_type_ = actuator_type_;

  Eigen::Affine3d pos_af3 = tf2::transformToEigen(shared_data_->robot_pose_);
  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
  double eps = 1e-4;
  traj.cost_ = 0.0; // placed here in case we return early
  traj.resetPoses();
  const auto& measured = shared_data_->robot_state_.twist.twist;
  Eigen::Vector3f initial_velocity(measured.linear.x, measured.linear.y, measured.angular.z);
  const Eigen::Vector3f predicted_target = predictedBodyVelocity(sample_target_vel);
  if (!predicted_target.allFinite()) return false;
  vmag=std::max(vmag,std::hypot(static_cast<double>(predicted_target[0]),
                               static_cast<double>(predicted_target[1])));
  if (!sample_target_vel.allFinite() || !initial_velocity.allFinite() ||
      sample_target_vel[0] < limits_.min_vel_x - eps ||
      sample_target_vel[0] > limits_.max_vel_x + eps ||
      sample_target_vel[1] < limits_.min_vel_y - eps ||
      sample_target_vel[1] > limits_.max_vel_y + eps ||
      std::abs(sample_target_vel[2]) > limits_.max_vel_theta + eps) return false;
  const Eigen::Vector3f acceleration = limits_.getAccLimits();
  if (!acceleration.allFinite() || (acceleration.array() <= 0).any() ||
      !std::isfinite(limits_.deceleration_ratio) || limits_.deceleration_ratio <= 0 ||
      !std::isfinite(params_.sim_time) || params_.sim_time <= 0 ||
      !std::isfinite(params_.sim_granularity) || params_.sim_granularity <= 0 ||
      !std::isfinite(params_.angular_sim_granularity) || params_.angular_sim_granularity <= 0 ||
      !std::isfinite(params_.controller_frequency) || params_.controller_frequency <= 0) return false;

  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Trajectory by state x: %.2f, y: %.2f, w: %.2f", sample_target_vel[0], sample_target_vel[1], sample_target_vel[2]);

  const bool braking_stop = sample_braking_commands_ && sample_target_vel.isZero(0.0f);
  if (!braking_stop && (limits_.min_vel_trans >= 0 && vmag + eps < limits_.min_vel_trans) &&
      (limits_.min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_.min_vel_theta)) {
    return false;
  }
  if (limits_.max_vel_trans >=0 && vmag - eps > limits_.max_vel_trans) {
    return false;
  }

  if(shared_data_->current_allowed_max_linear_speed_>0.0){
    if (vmag - eps > shared_data_->current_allowed_max_linear_speed_) {
      return false;
    }
  }

  int num_steps;

  double sim_time_distance = std::hypot(
      std::max(std::abs(initial_velocity[0]), std::abs(predicted_target[0])),
      std::max(std::abs(initial_velocity[1]), std::abs(predicted_target[1]))) * params_.sim_time;
  double sim_time_angle = std::max(std::abs(initial_velocity[2]), std::abs(predicted_target[2])) * params_.sim_time;
  num_steps =
      ceil(std::max(sim_time_distance / params_.sim_granularity,
          sim_time_angle / params_.angular_sim_granularity));

  if (num_steps == 0 && !braking_stop) {
    return false;
  }
  num_steps = std::max(2, num_steps);
  num_steps = std::max(num_steps, static_cast<int>(std::ceil(params_.sim_time * params_.controller_frequency)));

  const double dt = params_.sim_time / num_steps;
  const int command_steps = num_steps;
  int reaction_steps = 0;
  if (sample_braking_commands_) {
    reaction_steps = static_cast<int>(std::ceil(braking_reaction_time_ / dt));
    double stopping_time = 0;
    for (int axis=0; axis<3; ++axis)
      stopping_time = std::max(stopping_time,
        static_cast<double>(std::max(std::abs(initial_velocity[axis]), std::abs(predicted_target[axis])) /
        (acceleration[axis] * limits_.deceleration_ratio)));
    num_steps += reaction_steps + static_cast<int>(std::ceil(stopping_time / dt));
  }
  traj.time_delta_ = dt;

  Eigen::Vector3f loop_vel;

  loop_vel = initial_velocity;
  traj.xv_     = sample_target_vel[0];
  traj.yv_     = sample_target_vel[1];
  traj.thetav_ = sample_target_vel[2];

  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  for (int i = 0; i <= num_steps; ++i) {
    if (i > 0) {
      Eigen::Vector3f average_velocity;
      const int step = i-1;
      for (int axis=0; axis<3; ++axis) {
        double velocity=loop_vel[axis];
        if (step < reaction_steps) {
          average_velocity[axis]=velocity;
        } else {
          const double target = step < reaction_steps+command_steps ? predicted_target[axis] : 0.0;
          average_velocity[axis]=integrateVelocity(velocity, target, acceleration[axis],
                                                  limits_.deceleration_ratio, dt)/dt;
          loop_vel[axis]=velocity;
        }
      }
      pos = computeNewPositions(pos, average_velocity, dt);
    }

    Eigen::Affine3d trans_gbl2traj_af3;

    Eigen::Affine3d trans_b2traj_af3(Eigen::AngleAxisd(pos[2], Eigen::Vector3d::UnitZ()));
    trans_b2traj_af3.translation().x() = pos[0];
    trans_b2traj_af3.translation().y() = pos[1];

    trans_gbl2traj_af3 = pos_af3*trans_b2traj_af3;
    geometry_msgs::msg::TransformStamped trans_gbl2traj_ = tf2::eigenToTransform (trans_gbl2traj_af3);
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header = shared_data_->robot_pose_.header;
    ros_pose.pose.position.x = trans_gbl2traj_.transform.translation.x;
    ros_pose.pose.position.y = trans_gbl2traj_.transform.translation.y;
    ros_pose.pose.position.z = trans_gbl2traj_.transform.translation.z;
    ros_pose.pose.orientation = trans_gbl2traj_.transform.rotation;

    pcl::PointCloud<pcl::PointXYZ> pc_out;
    pcl::transformPointCloud(params_.cuboid, pc_out, trans_gbl2traj_af3);

    base_trajectory::cuboid_min_max_t cuboid_min_max;
    pcl::getMinMax3D(pc_out, cuboid_min_max.first, cuboid_min_max.second);

    if(!traj.addPoseCuboid(ros_pose, pc_out, cuboid_min_max)){
      return false;
    }

  } // end for simulation steps

  return true; // trajectory has at least one point
}

Eigen::Vector3f CoupledPathTrajectoryGeneratorTheory::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

}  // namespace trajectory_generators
