#include <p2p_move_base/p2p_move_base.h>
#include <dddmr_sys_core/motion_timestamp.h>
#include <tf2/utils.h>
#include <sstream>
#include <iomanip>

namespace p2p_move_base {
// Uses the main generator's actual turnRate, rollout, critics and settling state.
// No global route is executed, and no forward command is permitted on this endpoint.
void P2PMoveBase::executeRotationTest(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> handle) {
  using Action=dddmr_sys_core::action::PToPMoveBase;
  auto cleanup=std::shared_ptr<void>(nullptr,[this](void*) {
    publishZeroVelocity("旋转测试结束");
    if(LP_->traj_shared_data_) LP_->traj_shared_data_->rotation_test_=false;
    LP_->resetRotationReference();task_running_=false;
  });
  auto result=std::make_shared<Action::Result>();
  auto wrap=[](double x){return std::atan2(std::sin(x),std::cos(x));};
  auto started=std::chrono::steady_clock::now();
  auto elapsed=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();};
  std::string reason="timeout";
  bool success=false,have_start=false,final_fresh=false,settled=false;
  geometry_msgs::msg::TransformStamped start_pose,last_pose;
  nav_msgs::msg::Odometry start_odom,last_odom;
  double target=0,previous_yaw=0,turned=0;
  auto fresh=[&](const auto& pose,const auto& odom) {
    const auto now=clock_->now();
    const auto ps=rclcpp::Time(pose.header.stamp),os=rclcpp::Time(odom.header.stamp);
    const auto& v=odom.twist.twist;
    return dddmr_sys_core::motionTimestampFresh(ps.nanoseconds(),(now-ps).seconds()) &&
      dddmr_sys_core::motionTimestampFresh(os.nanoseconds(),(now-os).seconds()) &&
      std::isfinite(pose.transform.translation.x) && std::isfinite(pose.transform.translation.y) &&
      std::isfinite(pose.transform.translation.z) && std::isfinite(odom.pose.pose.position.x) &&
      std::isfinite(odom.pose.pose.position.y) && std::isfinite(odom.pose.pose.position.z) &&
      std::isfinite(tf2::getYaw(pose.transform.rotation)) &&
      std::isfinite(tf2::getYaw(odom.pose.pose.orientation)) &&
      std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.angular.z);
  };
  try {
    // Test is available only after explicit localization in persistent odom mode.
    if(!get_parameter("relocalization_only").as_bool() || !setOdomOnly(true))
      throw std::runtime_error("odom_mode_unavailable");
    GPM_->stop();rotation_pulse_.reset();LP_->resetRotationReference();
    STATE_->initialParams(LP_->getGlobalPose(),clock_->now());
    STATE_->setDecision("d_controlling"); // generator owns all turn/settle gates
    LP_->syncRobotState(start_odom,ackermann_drive_state_);start_pose=LP_->getGlobalPose();
    if(!fresh(start_pose,start_odom))throw std::runtime_error("stale_start_pose_or_odometry");
    if(std::hypot(start_odom.twist.twist.linear.x,start_odom.twist.twist.linear.y)>0.03 ||
       std::abs(start_odom.twist.twist.angular.z)>0.05)throw std::runtime_error("robot_not_stopped");
    have_start=true;last_pose=start_pose;last_odom=start_odom;
    target=wrap(tf2::getYaw(start_pose.transform.rotation)+handle->get_goal()->target_value);
    previous_yaw=tf2::getYaw(start_odom.pose.pose.orientation);
    rclcpp::WallRate rate(STATE_->controller_frequency_);
    while(rclcpp::ok() && elapsed()<40) {
      if(handle->is_canceling()){reason="canceled";break;}
      LP_->syncRobotState(last_odom,ackermann_drive_state_);last_pose=LP_->getGlobalPose();
      if(!fresh(last_pose,last_odom)){reason="stale_pose_or_odometry";break;}
      const double yaw=tf2::getYaw(last_odom.pose.pose.orientation);
      turned+=wrap(yaw-previous_yaw);previous_yaw=yaw;
      base_trajectory::Trajectory best;
      const auto status=LP_->computeVelocityCommand(STATE_->main_trajectory_generator_,best,true,target);
      auto feedback=std::make_shared<Action::Feedback>();feedback->base_position=last_pose;
      std::ostringstream trace;trace<<std::setprecision(9)<<"{\"elapsed\":"<<elapsed()
        <<",\"planner_state\":"<<static_cast<int>(status)<<",\"stage\":"<<LP_->traj_shared_data_->rotation_test_stage_
        <<",\"command_w\":"<<(status==dddmr_sys_core::TRAJECTORY_FOUND && best.cost_>=0?best.thetav_:0)<<",\"measured_w\":"<<last_odom.twist.twist.angular.z
        <<",\"turned_rad\":"<<turned<<"}";
      feedback->last_decision=LP_->rotation_test_diagnostics_;
      feedback->current_decision=trace.str();handle->publish_feedback(feedback);
      if(status!=dddmr_sys_core::TRAJECTORY_FOUND){reason="planner_rejected_"+std::to_string(static_cast<int>(status));break;}
      if(std::abs(best.xv_)>1e-6 || std::abs(best.yv_)>1e-6){reason="non_rotation_command_rejected";break;}
      if(LP_->traj_shared_data_->rotation_test_complete_){success=true;reason="angle_reached";break;}
      publishVelocity(best);rate.sleep();
    }
  } catch(const std::exception& e) {
    // Stable strings above are JSON safe; exception details remain in ROS logs.
    reason=e.what();
    if(reason!="odom_mode_unavailable" && reason!="stale_start_pose_or_odometry" && reason!="robot_not_stopped")
      reason="test_error";
    RCLCPP_ERROR(get_logger(),"Rotation test error: %s",e.what());
  }
  // Always brake, including cancellation/rejection; final measurement is after stopping.
  unsigned stopped=0;int64_t last_stamp=0;
  const double stop_started=elapsed();
  rclcpp::WallRate stop_rate(20);
  while(rclcpp::ok() && elapsed()-stop_started<5) {
    publishZeroVelocity("旋转测试停车并记录结束位姿");
    nav_msgs::msg::Odometry odom;ackermann_msgs::msg::AckermannDriveStamped drive;
    LP_->syncRobotState(odom,drive);const auto pose=LP_->getGlobalPose();
    final_fresh=fresh(pose,odom);
    if(final_fresh && have_start) {
      const double yaw=tf2::getYaw(odom.pose.pose.orientation);turned+=wrap(yaw-previous_yaw);previous_yaw=yaw;
      last_pose=pose;last_odom=odom;
      const auto stamp=rclcpp::Time(odom.header.stamp).nanoseconds();
      if(std::hypot(odom.twist.twist.linear.x,odom.twist.twist.linear.y)<=0.03 && std::abs(odom.twist.twist.angular.z)<=0.05) {
        if(stamp>last_stamp)++stopped;
      } else stopped=0;
      last_stamp=stamp;
      if(stopped>=3){settled=true;break;}
    } else stopped=0;
    stop_rate.sleep();
  }
  if(!settled){success=false;reason+="_stop_unconfirmed";}
  if(handle->is_canceling()){success=false;reason="canceled";}
  auto pose_json=[](const auto& p){std::ostringstream s;s<<std::setprecision(10)<<"["<<p.position.x<<","<<p.position.y<<","<<p.position.z<<","<<tf2::getYaw(p.orientation)<<"]";return s.str();};
  auto map_pose=[](const auto& t){geometry_msgs::msg::Pose p;p.position.x=t.transform.translation.x;p.position.y=t.transform.translation.y;p.position.z=t.transform.translation.z;p.orientation=t.transform.rotation;return p;};
  std::ostringstream report;report<<std::setprecision(10)<<"{\"reason\":\""<<reason<<"\",\"success\":"<<(success?"true":"false")
    <<",\"settled\":"<<(settled?"true":"false")<<",\"final_fresh\":"<<(final_fresh?"true":"false")
    <<",\"requested_rad\":"<<handle->get_goal()->target_value<<",\"duration\":"<<elapsed();
  if(have_start) report<<",\"start_map\":"<<pose_json(map_pose(start_pose))<<",\"end_map\":"<<pose_json(map_pose(last_pose))
    <<",\"start_odom\":"<<pose_json(start_odom.pose.pose)<<",\"end_odom\":"<<pose_json(last_odom.pose.pose)<<",\"turned_rad\":"<<turned;
  report<<"}";result->status=success?1:2;result->result=report.str();
  RCLCPP_INFO(get_logger(),"Rotation test result: %s",result->result.c_str());
  if(handle->is_canceling())handle->canceled(result);else if(success)handle->succeed(result);else handle->abort(result);
}
}
