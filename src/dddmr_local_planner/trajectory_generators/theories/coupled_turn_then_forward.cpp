#include <trajectory_generators/coupled_path_trajectory_generator_theory.h>
#include <trajectory_generators/braking_rollout.h>
#include <pcl/common/common.h>
#include <dddmr_sys_core/motion_timestamp.h>

namespace trajectory_generators {
void CoupledPathTrajectoryGeneratorTheory::resetTurnState() {
  turn_phase_=TurnPhase::Brake;
  turn_locked_=TurnCandidate{};
  turn_stopped_count_=0;
  turn_sign_=0;
  turn_last_stamp_=0;
  turn_started_=0;
}

double CoupledPathTrajectoryGeneratorTheory::turnRate(double error) const {
  if (std::abs(error)<=axis_yaw_exit_) return 0;
  double rate=std::min(limits_.max_vel_theta,
    tracker_.config().yaw_gain*std::abs(deadband(error,tracker_.config().yaw_deadband)));
  // 纯旋转可能产生模型预测的 X 漂移，也必须遵守平移限速。
  const double k=std::abs(tracker_.config().k_xw);
  double limit=limits_.max_vel_trans;
  if (shared_data_->current_allowed_max_linear_speed_>0)
    limit=std::min(limit,shared_data_->current_allowed_max_linear_speed_);
  if (k>1e-9) rate=std::min(rate,limit/k);
  return std::copysign(rate,error);
}

CoupledPathTrajectoryGeneratorTheory::TurnKey
CoupledPathTrajectoryGeneratorTheory::turnKey(const base_trajectory::Trajectory& t) {
  const auto end=t.getPose(t.getPosesSize()-1).pose;
  const auto& q=end.orientation;
  const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  return {t.xv_,t.yv_,t.thetav_,end.position.x,end.position.y,yaw};
}

void CoupledPathTrajectoryGeneratorTheory::prepareTurnCandidates(
    const Reference& ref,double x,double y,double yaw) {
  const auto now=node_->now().nanoseconds();
  if (turn_epoch_!=shared_data_->rotation_reference_epoch_ || turn_last_selection_<=0 ||
      now<turn_last_selection_ || now-turn_last_selection_>500000000 ||
      cycle_stamp_<turn_last_stamp_) resetTurnState();
  turn_epoch_=shared_data_->rotation_reference_epoch_;
  turn_position_={x,y};turn_yaw_=yaw;turn_goal_=ref.lookahead;
  const double dx=turn_goal_[0]-x,dy=turn_goal_[1]-y;
  turn_desired_heading_=std::hypot(dx,dy)>1e-6?std::atan2(dy,dx):yaw;
  if (shared_data_->rotation_test_) {
    shared_data_->rotation_test_supported_=true;
    turn_desired_heading_=shared_data_->rotation_test_heading_;
  }
  // 直行期间不边走边转。路线转弯或走完当前短段时，先停车再选择新方向。
  if (turn_phase_==TurnPhase::Drive &&
      (std::abs(wrap(turn_desired_heading_-turn_locked_.heading))>axis_yaw_enter_ ||
       std::abs(wrap(yaw-turn_locked_.heading))>axis_yaw_exit_ ||
       std::abs(cycle_measured_[2])>0.05 || std::abs(cycle_measured_[1])>0.03 ||
       std::hypot(x-turn_segment_start_[0],y-turn_segment_start_[1])>=tracker_.config().lookahead))
    resetTurnState();
  if ((turn_phase_==TurnPhase::Turn || turn_phase_==TurnPhase::Settle) &&
      turn_started_>0 && static_cast<double>(now-turn_started_)*1e-9>turn_timeout_) {
    RCLCPP_WARN(node_->get_logger(),"%s turn_then_forward: turn timeout, brake before replanning direction",name_.c_str());
    resetTurnState();
  }
  const double remaining_angle=wrap(turn_locked_.heading-yaw);
  if (turn_phase_==TurnPhase::Turn && turn_sign_!=0 &&
      std::abs(remaining_angle)>axis_yaw_exit_ && (remaining_angle>0?1:-1)!=turn_sign_) {
    // 转过头需要反向时，先完成停车确认，不能直接反打角速度。
    turn_phase_=TurnPhase::Settle;turn_stopped_count_=0;turn_sign_=0;
  }
  // 到角先切停稳，避免旋转测试的零速目标与停车候选指纹重复。
  if (turn_phase_==TurnPhase::Turn && std::abs(remaining_angle)<=axis_yaw_exit_) {
    turn_phase_=TurnPhase::Settle;turn_stopped_count_=0;
  }
  if (shared_data_->rotation_test_ && turn_phase_==TurnPhase::Search &&
      std::abs(wrap(turn_desired_heading_-yaw))<=axis_yaw_exit_) {
    turn_locked_={turn_desired_heading_,0.0,false};turn_started_=now;
    turn_phase_=TurnPhase::Settle;turn_stopped_count_=0;
  }
  turn_candidates_.push_back(TurnCandidate{}); // 始终保留真实制动轨迹。
  if (turn_phase_==TurnPhase::Brake || turn_phase_==TurnPhase::Settle) return;
  if (shared_data_->rotation_test_) {
    if (turn_phase_==TurnPhase::Drive) return;
    turn_candidates_.push_back({turn_desired_heading_,0.0,false});
    return;
  }
  double max_speed=std::min({limits_.max_vel_x,limits_.max_vel_trans,tracker_.config().cruise_speed,
    tracker_.config().approach_gain*std::hypot(dx,dy)});
  if (shared_data_->current_allowed_max_linear_speed_>0)
    max_speed=std::min(max_speed,shared_data_->current_allowed_max_linear_speed_);
  if (max_speed<1e-6) return;
  std::vector<double> headings;
  auto add_heading=[&](double heading) {
    heading=wrap(heading);
    for (double old:headings) if (std::abs(wrap(old-heading))<1e-6) return;
    headings.push_back(heading);
  };
  if (turn_phase_==TurnPhase::Search) {
    for(int i=0;i<params_.angular_z_sample;++i) {
      const double offset=params_.angular_z_sample==1?0:
        -turn_angle_range_+2*turn_angle_range_*i/(params_.angular_z_sample-1);
      add_heading(yaw+offset);
    }
    add_heading(yaw);
    add_heading(turn_desired_heading_);
  } else add_heading(turn_locked_.heading);
  for (double heading:headings) {
    // 不选朝离开前视目标方向直行的候选。
    if (std::cos(wrap(heading-turn_desired_heading_))<=0) continue;
    for(int i=1;i<=params_.linear_x_sample;++i)
      turn_candidates_.push_back({heading,max_speed*i/params_.linear_x_sample,false});
  }
}

bool CoupledPathTrajectoryGeneratorTheory::generateTurnTrajectory(
    const TurnCandidate& c,base_trajectory::Trajectory& traj) {
  if(c.stop) return generateTrajectory(Eigen::Vector3f::Zero(),traj);
  const bool drive=turn_phase_==TurnPhase::Drive ||
    std::abs(wrap(c.heading-turn_yaw_))<=axis_yaw_exit_;
  if (drive && shared_data_->rotation_test_) return generateTrajectory(Eigen::Vector3f::Zero(),traj);
  const double angular_command=drive?0:turnRate(wrap(c.heading-turn_yaw_));
  // 当前周期仅下发第一阶段指令，未来阶段由停稳/角度反馈确认后再下发。
  traj.xv_=drive?c.speed:0;traj.yv_=0;traj.thetav_=angular_command;
  traj.actuator_type_=actuator_type_;traj.cost_=0;traj.resetPoses();
  const Eigen::Affine3d origin=tf2::transformToEigen(shared_data_->robot_pose_);
  const Eigen::Vector3f acc=limits_.getAccLimits();
  Eigen::Vector3f velocity(cycle_measured_[0],cycle_measured_[1],cycle_measured_[2]);
  Eigen::Vector3f pos=Eigen::Vector3f::Zero();
  const double dt=std::min({0.05,1.0/params_.controller_frequency,
    params_.angular_sim_granularity/std::max(limits_.max_vel_theta,std::abs(cycle_measured_[2])),
    params_.sim_granularity/std::max({0.01,limits_.max_vel_trans,
      std::hypot(cycle_measured_[0],cycle_measured_[1])})});
  if (!std::isfinite(dt) || dt<=0) return false;
  traj.time_delta_=dt;
  auto append=[&]() {
    Eigen::Affine3d relative(Eigen::AngleAxisd(pos[2],Eigen::Vector3d::UnitZ()));
    relative.translation().x()=pos[0];relative.translation().y()=pos[1];
    const Eigen::Affine3d world=origin*relative;
    const auto transform=tf2::eigenToTransform(world);
    geometry_msgs::msg::PoseStamped pose;pose.header=shared_data_->robot_pose_.header;
    pose.pose.position.x=transform.transform.translation.x;
    pose.pose.position.y=transform.transform.translation.y;
    pose.pose.position.z=transform.transform.translation.z;
    pose.pose.orientation=transform.transform.rotation;
    pcl::PointCloud<pcl::PointXYZ> cuboid;pcl::transformPointCloud(params_.cuboid,cuboid,world);
    base_trajectory::cuboid_min_max_t bounds;pcl::getMinMax3D(cuboid,bounds.first,bounds.second);
    return traj.addPoseCuboid(pose,cuboid,bounds);
  };
  if(!append())return false;
  auto integrate=[&](const Eigen::Vector3f& target,bool reaction) {
    Eigen::Vector3f average;
    for(int axis=0;axis<3;++axis){
      double value=velocity[axis];
      average[axis]=reaction?value:integrateVelocity(value,target[axis],acc[axis],limits_.deceleration_ratio,dt)/dt;
      velocity[axis]=value;
    }
    pos=computeNewPositions(pos,average,dt);
    return append();
  };
  for(int i=0;i<static_cast<int>(std::ceil(braking_reaction_time_/dt));++i)
    if(!integrate(Eigen::Vector3f::Zero(),true))return false;
  if(!drive) {
    bool settled=false,braking=false;
    int reaction_left=0;
    for(int i=0;i<static_cast<int>(std::ceil(turn_timeout_/dt));++i) {
      const double error=wrap(c.heading-turn_yaw_-pos[2]);
      if(!braking && std::abs(error)<=axis_yaw_exit_) {
        braking=true;
        reaction_left=static_cast<int>(std::ceil(braking_reaction_time_/dt));
      }
      if(braking) {
        // 角度进入容差后，计入停车反应延迟和制动漂移再复核朝向。
        if(!integrate(Eigen::Vector3f::Zero(),reaction_left>0))return false;
        if(reaction_left>0)--reaction_left;
        else if(velocity.cwiseAbs().maxCoeff()<1e-5) {
          if(std::abs(wrap(c.heading-turn_yaw_-pos[2]))<=axis_yaw_exit_) {settled=true;break;}
          braking=false;
        }
      } else {
        const Eigen::Vector3f target=predictedBodyVelocity(Eigen::Vector3f(0,0,turnRate(error)));
        if(!integrate(target,false))return false;
      }
    }
    if(!settled)return false;
  }
  // 测试复用完整转向/制动预测，到此结束，不预测或执行后续前进。
  if (shared_data_->rotation_test_) return true;
  // 无持续 yaw/Y 指令；初始测量中的残余运动仍真实积分，不伪造瞬间停稳。
  const Eigen::Vector3f forward(c.speed,0,0);
  for(int i=0;i<static_cast<int>(std::ceil(params_.sim_time/dt));++i)
    if(!integrate(forward,false))return false;
  double stop_time=0;
  for(int axis=0;axis<3;++axis)
    stop_time=std::max(stop_time,std::abs(static_cast<double>(velocity[axis]))/(acc[axis]*limits_.deceleration_ratio));
  for(int i=0;i<std::ceil(stop_time/dt)+1;++i)
    if(!integrate(Eigen::Vector3f::Zero(),false))return false;
  return true;
}

void CoupledPathTrajectoryGeneratorTheory::selectTurnTrajectory(
    const std::vector<base_trajectory::Trajectory>& accepted,base_trajectory::Trajectory& best) {
  best.cost_=-1;
  const auto now=node_->now().nanoseconds();
  turn_last_selection_=now;
  if(!cycle_valid_ || !dddmr_sys_core::motionTimestampFresh(cycle_stamp_,
      static_cast<double>(now-cycle_stamp_)*1e-9)) {resetTurnState();return;}
  const bool stopped=std::hypot(cycle_measured_[0],cycle_measured_[1])<=0.03 && std::abs(cycle_measured_[2])<=0.05;
  if (turn_phase_==TurnPhase::Search && !stopped) resetTurnState();
  if(!stopped)turn_stopped_count_=0;
  else if(cycle_stamp_>turn_last_stamp_)++turn_stopped_count_;
  turn_last_stamp_=cycle_stamp_;
  const base_trajectory::Trajectory* brake=nullptr;
  const base_trajectory::Trajectory* selected=nullptr;
  TurnCandidate plan;
  double score=std::numeric_limits<double>::infinity();
  for(const auto& t:accepted) {
    if(!std::isfinite(t.cost_) || t.cost_<0 || t.getPosesSize()<2)continue;
    const auto found=turn_generated_.find(turnKey(t));
    if(found==turn_generated_.end())continue;
    const auto& candidate=found->second;
    if(candidate.stop){brake=&t;continue;}
    const auto endpoint=t.getPose(t.getPosesSize()-1).pose.position;
    // 相同当前指令可能对应不同未来方向，按整条轨迹代价及目标进展选择。
    const double cost=t.cost_+std::hypot(endpoint.x-turn_goal_[0],endpoint.y-turn_goal_[1])+
      std::abs(wrap(candidate.heading-turn_desired_heading_));
    if(cost<score){score=cost;selected=&t;plan=candidate;}
  }
  if(turn_phase_==TurnPhase::Brake || turn_phase_==TurnPhase::Settle) {
    if(brake)best=*brake;
    if(brake && turn_stopped_count_>=3) {
      if(turn_phase_==TurnPhase::Brake)turn_phase_=TurnPhase::Search;
      else if(std::abs(wrap(turn_locked_.heading-turn_yaw_))<=axis_yaw_exit_) {
        turn_phase_=TurnPhase::Drive;turn_segment_start_=turn_position_;
      } else turn_phase_=TurnPhase::Turn;
      turn_stopped_count_=0;
    }
  } else if(shared_data_->rotation_test_ && turn_phase_==TurnPhase::Drive) {
    if(brake)best=*brake;
  } else if(selected) {
    best=*selected;
    if(std::abs(best.thetav_)>1e-6)turn_sign_=best.thetav_>0?1:-1;
    if(turn_phase_==TurnPhase::Search) {
      turn_locked_=plan;turn_started_=now;turn_segment_start_=turn_position_;
      turn_phase_=std::abs(best.thetav_)>1e-6?TurnPhase::Turn:TurnPhase::Drive;
    } else if(turn_phase_==TurnPhase::Turn && std::abs(wrap(plan.heading-turn_yaw_))<=axis_yaw_exit_) {
      // 未停稳之前，只能选择已通过检查的停车轨迹，不发送尚未准入的前进指令。
      best.cost_=-1;if(brake)best=*brake;
      turn_phase_=TurnPhase::Settle;turn_stopped_count_=0;
    }
  } else {
    if(brake)best=*brake;
    resetTurnState();
  }
  if (shared_data_->rotation_test_) {
    shared_data_->rotation_test_stage_=static_cast<int>(turn_phase_);
    shared_data_->rotation_test_complete_=turn_phase_==TurnPhase::Drive && stopped &&
      std::abs(wrap(turn_desired_heading_-turn_yaw_))<=axis_yaw_exit_;
  }
  if(tracking_diagnostics_) RCLCPP_INFO(node_->get_logger(),
    "%s tracking_diag cycle=%llu phase=turn_forward stage=%d candidates=%zu accepted=%zu "
    "locked_heading=%.4f yaw_error=%.4f selected_valid=%d command=(%.4f,%.4f,%.4f)",
    name_.c_str(),static_cast<unsigned long long>(diagnostic_cycle_),static_cast<int>(turn_phase_),
    turn_candidates_.size(),accepted.size(),turn_locked_.heading,wrap(turn_locked_.heading-turn_yaw_),
    best.cost_>=0,best.cost_>=0?best.xv_:0,best.cost_>=0?best.yv_:0,best.cost_>=0?best.thetav_:0);
}
} // namespace trajectory_generators
