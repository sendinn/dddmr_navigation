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
#include <p2p_move_base/p2p_move_base.h>

namespace p2p_move_base
{

P2PMoveBase::P2PMoveBase(std::string name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
  rotation_pulse_duration_ = declare_parameter<double>("rotation_pulse_duration", 0.0);
  if (!std::isfinite(rotation_pulse_duration_) || rotation_pulse_duration_ < 0)
    throw std::invalid_argument("Invalid rotation_pulse_duration");
  rotation_predict_duration_ = declare_parameter<bool>("rotation_predict_duration", false);
  rotation_angle_feedback_ = declare_parameter<bool>("rotation_angle_feedback", false);
  rotation_feedback_timeout_ = declare_parameter<double>("rotation_feedback_timeout", 10.0);
  if (!std::isfinite(rotation_feedback_timeout_) || rotation_feedback_timeout_ <= 0)
    throw std::invalid_argument("Invalid rotation_feedback_timeout");
  rotation_calibration_angle_ = declare_parameter<double>("rotation_calibration_angle", 0.25051551822739304);
  rotation_calibration_time_ = declare_parameter<double>("rotation_calibration_time", 0.5);
  rotation_max_duration_ = declare_parameter<double>("rotation_max_duration", 2.0);
  if (!std::isfinite(rotation_calibration_angle_) || rotation_calibration_angle_ <= 0 ||
      !std::isfinite(rotation_calibration_time_) || rotation_calibration_time_ <= 0 ||
      !std::isfinite(rotation_max_duration_) || rotation_max_duration_ <= 0)
    throw std::invalid_argument("Invalid rotation calibration");
  declare_parameter<bool>("use_mcl_during_navigation", true);
  odom_only_client_ = create_client<std_srvs::srv::SetBool>("mcl/set_odom_only");
  localization_timeout_ = declare_parameter<double>("localization_timeout", 0.0);
  localization_resume_stable_time_ = declare_parameter<double>("localization_resume_stable_time", 1.0);
  if (!std::isfinite(localization_resume_stable_time_) || localization_resume_stable_time_ <= 0.0)
    throw std::invalid_argument("localization_resume_stable_time must be positive");
  const double xy_limit = declare_parameter<double>("localization_xy_std_max", 0.15);
  const double yaw_limit = declare_parameter<double>("localization_yaw_std_max", 0.20);
  localization_xy_limit_ = xy_limit;
  localization_yaw_limit_ = yaw_limit;
  if (!std::isfinite(localization_timeout_) || localization_timeout_ < 0.0 ||
      !std::isfinite(xy_limit) || xy_limit <= 0.0 || !std::isfinite(yaw_limit) || yaw_limit <= 0.0)
    throw std::invalid_argument("Invalid localization safety thresholds");
  if (localization_timeout_ > 0.0) {
    localization_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "mcl_pose", rclcpp::QoS(1).best_effort(),
      [this, xy_limit, yaw_limit](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(localization_diagnostics_mutex_);
        const auto& c = msg->pose.covariance;
        localization_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
        localization_cov_x_ = c[0];
        localization_cov_y_ = c[7];
        localization_cov_yaw_ = c[35];
        const double age = (clock_->now() - rclcpp::Time(msg->header.stamp)).seconds();
        const bool valid = std::isfinite(c[0]) && c[0] >= 0 && c[0] <= xy_limit * xy_limit &&
          std::isfinite(c[7]) && c[7] >= 0 && c[7] <= xy_limit * xy_limit &&
          std::isfinite(c[35]) && c[35] >= 0 && c[35] <= yaw_limit * yaw_limit &&
          msg->header.stamp.sec > 0 && age >= 0 && age < localization_timeout_;
        const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        localization_recovery_.observe(valid, now, localization_stamp_ns_, localization_valid_until_.load());
        localization_valid_until_.store(valid ? now + static_cast<int64_t>(
            (localization_timeout_ - age) * 1e9) : 0);
      });
  }
  enable_rotate_recovery_ = declare_parameter<bool>("enable_rotate_recovery", true);
  stop_after_heading_alignment_ = declare_parameter<bool>("stop_after_heading_alignment", false);
  RCLCPP_INFO(get_logger(), "enable_rotate_recovery: %s",
      enable_rotate_recovery_ ? "true" : "false");
}

rclcpp_action::GoalResponse P2PMoveBase::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::PToPMoveBase::Goal> goal)
{
  (void)uuid;
  if (task_running_.exchange(true)) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse P2PMoveBase::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void P2PMoveBase::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{

  if (is_active(current_handle_)){
    RCLCPP_INFO(this->get_logger(), "An older goal is active, cancelling current one.");
    auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
    current_handle_->abort(result);
    return;
  }
  else{
    current_handle_ = goal_handle;
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&P2PMoveBase::executeCb, this, std::placeholders::_1), goal_handle}.detach();
}

void P2PMoveBase::initial(const std::shared_ptr<local_planner::Local_Planner>& lp
                    ,const std::shared_ptr<p2p_move_base::P2PGlobalPlanManager>& gpm){
  
  LP_ = lp;
  GPM_ = gpm;

  STATE_ = std::make_shared<p2p_move_base::State>(this->get_node_logging_interface(), this->get_node_parameters_interface());
  
  if(STATE_->use_twist_stamped_){
    stamped_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 1);
  }
  else{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }
  stamped_ackermann_drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_drive_cmd", 1);

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
  
  recovery_behaviors_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  recovery_behaviors_client_ptr_ = rclcpp_action::create_client<dddmr_sys_core::action::RecoveryBehaviors>(
      this,
      "recovery_behaviors", recovery_behaviors_client_group_);

  //@Create action server
  action_server_p2p_move_base_ = rclcpp_action::create_server<dddmr_sys_core::action::PToPMoveBase>(
    this,
    "/p2p_move_base",
    std::bind(&P2PMoveBase::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&P2PMoveBase::handle_cancel, this, std::placeholders::_1),
    std::bind(&P2PMoveBase::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);

  RCLCPP_INFO(this->get_logger(), "\033[1;32m---->\033[0m P2P move base launched.");

}

P2PMoveBase::~P2PMoveBase(){
  STATE_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  LP_.reset();
  GPM_.reset();
}

bool P2PMoveBase::isQuaternionValid(const geometry_msgs::msg::Quaternion& q){
  //first we need to check if the quaternion has nan's or infs
  if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
    RCLCPP_ERROR(this->get_logger(), "Quaternion has nans or infs... discarding as a navigation goal");
    return false;
  }

  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  //next, we need to check if the length of the quaternion is close to zero
  if(tf_q.length2() < 1e-6){
    RCLCPP_ERROR(this->get_logger(), "Quaternion has length close to zero... discarding as navigation goal");
    return false;
  }

  //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
  tf_q.normalize();

  tf2::Vector3 up(0, 0, 1);

  double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

  if(fabs(dot - 1) > 1e-3){
    RCLCPP_ERROR(this->get_logger(), "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
    return false;
  }

  return true;
}

void P2PMoveBase::publishZeroVelocity(const char* reason, int source_line){
  RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
    "停车原因：%s; state=%s, source=p2p_move_base.cpp:%d",
    reason, STATE_->getCurrentDecision().c_str(), source_line);
  if (rotation_pulse_.active) rotation_pulse_.stop(
    std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  if(STATE_->use_twist_stamped_){
    geometry_msgs::msg::TwistStamped stamped_cmd_vel;
    stamped_cmd_vel.header.frame_id = LP_->getControlFrame();
    stamped_cmd_vel.header.stamp = clock_->now();
    stamped_cmd_vel.twist = cmd_vel;
    stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
  }
  else{
    cmd_vel_pub_->publish(cmd_vel);
  }

  ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_cmd;
  ackermann_drive_cmd.header.frame_id = LP_->getControlFrame();
  ackermann_drive_cmd.header.stamp = clock_->now();
  ackermann_drive_cmd.drive.speed = 0.0;
  ackermann_drive_cmd.drive.steering_angle = ackermann_drive_state_.drive.steering_angle;
  ackermann_drive_cmd.drive.steering_angle_velocity = 0.0;
  stamped_ackermann_drive_pub_->publish(ackermann_drive_cmd);
}

void P2PMoveBase::publishVelocity(const base_trajectory::Trajectory& cmd_traj){
  if (rotation_angle_feedback_ || rotation_predict_duration_ || rotation_pulse_duration_ > 0.0) {
    const bool rotating = std::abs(cmd_traj.thetav_) > 1e-6;
    const int sign = cmd_traj.thetav_ > 0 ? 1 : -1;
    const double now = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (rotation_pulse_.active && (!rotating || sign != rotation_pulse_.sign)) {
      publishZeroVelocity("旋转结束或方向切换，先制动");
      return;
    }
    if (rotating && !rotation_pulse_.active) {
      const auto stamp = rclcpp::Time(robot_state_.header.stamp);
      const double age = (clock_->now()-stamp).seconds();
      const auto& v = robot_state_.twist.twist;
      if (stamp.nanoseconds()<=0 || age<0 || age>=0.5 ||
          !std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) || !std::isfinite(v.angular.z) ||
          std::hypot(v.linear.x,v.linear.y)>0.03 || std::abs(v.angular.z)>0.05) {
        rotation_pulse_.stop(now);
        publishZeroVelocity("旋转前实测速度未停稳或里程计过期");
        return;
      }
      rotation_active_duration_ = rotation_angle_feedback_
        ? rotation_feedback_timeout_ : rotation_pulse_duration_;
      if (rotation_predict_duration_ && !rotation_angle_feedback_) {
        const double error = LP_->traj_shared_data_->rotation_error_;
        // Calibration applies only to the measured command magnitude.
        if (!std::isfinite(error) || error*sign <= 0 ||
            std::abs(std::abs(cmd_traj.thetav_)-0.6)>1e-4) {
          publishZeroVelocity();
          RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
            "Rotation prediction rejected: requires matching direction and 0.6 rad/s scored command.");
          return;
        }
        rotation_active_duration_ = predictedRotationDuration(error, rotation_calibration_angle_,
          rotation_calibration_time_, rotation_max_duration_);
        if (rotation_active_duration_ <= 0) { publishZeroVelocity(); return; }
        RCLCPP_INFO(get_logger(), "Predicted rotation: error=%.2f deg, command=%.3f rad/s, duration=%.3f s, maximum=%.2f s",
          error*180.0/std::acos(-1.0), cmd_traj.thetav_, rotation_active_duration_, rotation_max_duration_);
      }
      rotation_pulse_.start(now, sign);
    }
  }
  if (!progress_control_started_) {
    progress_control_started_ = true;
    STATE_->last_oscillation_reset_ = clock_->now();
    STATE_->oscillation_pose_ = LP_->getGlobalPose();
    RCLCPP_INFO(get_logger(), "Progress watchdog starts with first control command (not goal submission).");
  }

  if(cmd_traj.actuator_type_ == dddmr_sys_core::ActuatorType::MOTOR){
    // Report actual selected commands after the rotation/stop gates. Keep the
    // last moving phase across brief zero commands to avoid pulse-cycle spam.
    const int action = std::abs(cmd_traj.thetav_) > 1e-6 ? 1 :
      std::abs(cmd_traj.yv_) > 1e-6 ? 2 :
      cmd_traj.xv_ > 1e-6 ? 3 : cmd_traj.xv_ < -1e-6 ? 4 : 0;
    if (action == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
        "停车原因：局部规划器选中零速度轨迹；查看 trajectory_generators 的停车诊断; state=%s",
        STATE_->getCurrentDecision().c_str());
    }
    if (action != 0 && action != last_navigation_action_) {
      last_navigation_action_ = action;
      const char* label = action == 1 ? "旋转对方向中…" :
        action == 2 ? "横移回归路线中…" : action == 3 ? "前进中…" : "后退调整中…";
      RCLCPP_INFO(get_logger(), "Navigation action: %s (vx=%.3f m/s, vy=%.3f m/s, wz=%.3f rad/s)",
        label, cmd_traj.xv_, cmd_traj.yv_, cmd_traj.thetav_);
    }
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = cmd_traj.xv_;
    cmd_vel.linear.y = cmd_traj.yv_;
    cmd_vel.angular.z = cmd_traj.thetav_;
    if(STATE_->use_twist_stamped_){
      geometry_msgs::msg::TwistStamped stamped_cmd_vel;
      stamped_cmd_vel.header.frame_id = LP_->getControlFrame();
      stamped_cmd_vel.header.stamp = clock_->now();
      stamped_cmd_vel.twist = cmd_vel;
      stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
    }
    else{
      cmd_vel_pub_->publish(cmd_vel);
    }
  }
  else if(cmd_traj.actuator_type_ == dddmr_sys_core::ActuatorType::STEERING){
    ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_cmd;
    ackermann_drive_cmd.header.frame_id = LP_->getControlFrame();
    ackermann_drive_cmd.header.stamp = clock_->now();
    ackermann_drive_cmd.drive.speed = cmd_traj.xv_;
    ackermann_drive_cmd.drive.steering_angle = cmd_traj.steering_angle_;
    ackermann_drive_cmd.drive.steering_angle_velocity = cmd_traj.steering_angle_velocity_;
    stamped_ackermann_drive_pub_->publish(ackermann_drive_cmd);
  }
  else{
    RCLCPP_WARN(this->get_logger(), "Actuator type is not defined in Commanding Trajectory!, Robot will not going to move.");
  }
}

bool P2PMoveBase::setOdomOnly(bool enabled) {
  try {
    if (!odom_only_client_->wait_for_service(std::chrono::seconds(2))) return false;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = odom_only_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) return false;
    return future.get()->success;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "MCL mode service failed: %s", error.what());
    return false;
  }
}

void P2PMoveBase::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
  auto finish_task = std::shared_ptr<void>(nullptr, [this](void*) { task_running_ = false; });
  auto move_base_goal = goal_handle->get_goal();

  if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
    RCLCPP_WARN(this->get_logger(),"Aborting on goal because it was sent with an invalid quaternion");
    goal_handle->abort(result);
    publishZeroVelocity();
    return;
  }

  task_use_mcl_ = get_parameter("use_mcl_during_navigation").as_bool();
  // Restore scan matching on all exits: success, cancellation, rejection, or failure.
  auto restore_mcl = std::shared_ptr<void>(nullptr, [this, odom_only = !task_use_mcl_](void*) {
    if (odom_only) {
      publishZeroVelocity();
      if (!setOdomOnly(false))
        RCLCPP_ERROR(get_logger(), "MCL restore failed: reload navigation before the next task.");
    }
  });
  if (!task_use_mcl_) {
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (localization_timeout_ <= 0 || now >= localization_valid_until_.load() || !setOdomOnly(true)) {
      publishZeroVelocity();
      RCLCPP_ERROR(get_logger(), "Navigation mode rejected: valid initial localization and MCL mode service required.");
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(get_logger(), "Odometry-only navigation: map-to-odom frozen for this task.");
  }
  rclcpp::Rate r(STATE_->controller_frequency_);

  //@ if we dont initialize oscillation pose here, the first controlling entry will cause recovery behavior.
  //@ the rclcpp::Time initial are all done in FSM class
  STATE_->initialParams(LP_->getGlobalPose(), clock_->now());
  progress_control_started_ = false;
  last_navigation_action_ = 0;
  rotation_pulse_.reset();
  LP_->resetRotationReference();
  localization_paused_ = false;
  localization_replanning_ = false;
  STATE_->current_goal_ = move_base_goal->target_pose;
  GPM_->setGoal(STATE_->current_goal_);
  GPM_->resume();

  while(rclcpp::ok()){

    if(!goal_handle->is_active()){
      
      if(is_recoverying_.load()){
        RCLCPP_INFO(this->get_logger(), "P2P is in recovery state, cancel recovery behaviors.");
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
      }

      RCLCPP_INFO(this->get_logger(), "P2P move base preempted.");
      publishZeroVelocity();
      GPM_->stop();
      return;
    }

    if(goal_handle->is_canceling()){

      if(is_recoverying_.load()){
        RCLCPP_INFO(this->get_logger(), "P2P is in recovery state, cancel recovery behaviors.");
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
      }

      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "P2P move base cancelled.");
      publishZeroVelocity();
      GPM_->stop();
      return;
    }

    //the real work on pursuing a goal is done here
    bool done = executeCycle(goal_handle);
    
    auto feedback = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Feedback>();
    feedback->base_position = STATE_->global_pose_;
    feedback->last_decision = STATE_->getLastDecision();
    feedback->current_decision = STATE_->getCurrentDecision();
    goal_handle->publish_feedback(feedback);

    //if we're done, then we'll return from execute
    if(done){
      GPM_->stop();
      return;
    }
    
    r.sleep();

    //if(STATE_->isCurrentDecision("d_controlling") && r.cycleTime() > ros::Duration(1 / STATE_->controller_frequency_))
    //  ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", STATE_->controller_frequency_, r.cycleTime().toSec());
  }
  GPM_->stop();
}

bool P2PMoveBase::executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle){
    const auto steady_now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    bool localization_invalid = false;
    bool localization_recovered = false;
    if (task_use_mcl_ && localization_timeout_ > 0.0) {
      std::lock_guard<std::mutex> guard(localization_diagnostics_mutex_);
      localization_invalid = steady_now >= localization_valid_until_.load();
      localization_recovered = localization_recovery_.ready(
        steady_now, localization_valid_until_.load(), localization_resume_stable_time_);
      if (localization_invalid && !localization_paused_) {
        const double age = localization_stamp_ns_ > 0 ?
          (clock_->now().nanoseconds() - localization_stamp_ns_) * 1e-9 : -1.0;
        RCLCPP_ERROR(get_logger(),
          "Localization gate details: received=%d, age=%.3f s, timeout=%.3f s, "
          "cov_x=%.6f, cov_y=%.6f, xy_variance_limit=%.6f, "
          "cov_yaw=%.6f, yaw_variance_limit=%.6f",
          localization_stamp_ns_ > 0, age, localization_timeout_,
          localization_cov_x_, localization_cov_y_, localization_xy_limit_ * localization_xy_limit_,
          localization_cov_yaw_, localization_yaw_limit_ * localization_yaw_limit_);
      }
    }
    if (localization_invalid) {
      publishZeroVelocity();
      // Repeat cancellation until a pending recovery request has also stopped.
      if (is_recoverying_.load()) recovery_behaviors_client_ptr_->async_cancel_all_goals();
      if (!localization_paused_) {
        localization_paused_ = true;
        rotation_pulse_.reset();
        LP_->resetRotationReference();
        GPM_->pause();
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
        RCLCPP_WARN(get_logger(), "Localization paused: retaining goal; waiting for stable localization before replanning.");
      }
      return false;
    }
    if (localization_paused_) {
      publishZeroVelocity();
      if (is_recoverying_.load()) {
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
        return false;
      }
      if (!localization_recovered) return false;
      localization_replanning_ = true;
      localization_paused_ = false;
      STATE_->initialParams(LP_->getGlobalPose(), clock_->now());
      progress_control_started_ = false;
      heading_stopped_samples_ = 0;
      // A new request stamp forces a full plan even though the destination is unchanged.
      STATE_->current_goal_.header.stamp = clock_->now();
      GPM_->setGoal(STATE_->current_goal_);
      GPM_->resume();
      RCLCPP_INFO(get_logger(), "Localization recovered: replanning retained goal from current pose.");
      return false;
    }
    if (GPM_->planningUnsafe()) {
      publishZeroVelocity();
      recovery_behaviors_client_ptr_->async_cancel_all_goals();
      RCLCPP_ERROR(get_logger(), "Planning unavailable/stale: navigation aborted, cached local path disabled.");
      auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
      goal_handle->abort(result);
      return true;
    }

    STATE_->global_pose_ = LP_->getGlobalPose();
    LP_->syncRobotState(robot_state_, ackermann_drive_state_);
    if ((rotation_angle_feedback_ || rotation_predict_duration_ || rotation_pulse_duration_ > 0.0) && rotation_pulse_.active) {
      const auto stamp = rclcpp::Time(robot_state_.header.stamp);
      const auto& v = robot_state_.twist.twist;
      auto pulse = rotation_pulse_.poll(steady_now*1e-9, rotation_active_duration_,
        stamp.nanoseconds(), (clock_->now()-stamp).seconds(),
        std::hypot(v.linear.x,v.linear.y), v.angular.z, rotation_angle_feedback_);
      if (pulse == RotationPulse::Timeout) {
        publishZeroVelocity();
        RCLCPP_ERROR(get_logger(), "Rotation/settling timeout: aborting navigation.");
        goal_handle->abort(std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>());
        return true;
      }
      if (pulse == RotationPulse::Braking || pulse == RotationPulse::Settled) {
        publishZeroVelocity();
        if (pulse == RotationPulse::Settled) {
          LP_->resetRotationReference();
          STATE_->last_valid_control_ = clock_->now();
          RCLCPP_INFO(get_logger(), "Rotation pulse settled: recomputing heading and cross-track error.");
        }
        return false;
      }
    }

    // Gait-related vertical bobbing is not forward progress on the floor.
    const double progress_xy = std::hypot(
        STATE_->global_pose_.transform.translation.x - STATE_->oscillation_pose_.transform.translation.x,
        STATE_->global_pose_.transform.translation.y - STATE_->oscillation_pose_.transform.translation.y);
    if(progress_xy >= STATE_->oscillation_distance_ ||
          STATE_->getAngle(STATE_->global_pose_, STATE_->oscillation_pose_) >= STATE_->oscillation_angle_)
    {
      STATE_->oscillation_pose_ = STATE_->global_pose_;
      STATE_->last_oscillation_reset_ = clock_->now();
    }


    if(STATE_->isCurrentDecision("d_initial")){
      STATE_->setDecision("d_planning");
    }

    else if(STATE_->isCurrentDecision("d_planning")){
      GPM_->queryThread();
      STATE_->setDecision("d_planning_waitdone");
      return false;
    }

    else if(STATE_->isCurrentDecision("d_planning_waitdone")){
      
      //@If global planner keep return empty plan, we will enter this state for n seconds, then abort
      //@see: decision_planning
      std::vector<geometry_msgs::msg::PoseStamped> plan;
      if(GPM_->hasPlan()){
        GPM_->copyPlan(plan);
        if (localization_replanning_ && plan.size() < 3) {
          publishZeroVelocity();
          RCLCPP_ERROR(get_logger(), "Planning unavailable/stale: resumed plan has fewer than three points.");
          goal_handle->abort(std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>());
          return true;
        }
        //if the planner fails or returns a zero length plan, planning failed
        if(plan.empty()){
          RCLCPP_DEBUG(this->get_logger(), "Failed to find a plan to point (%.2f, %.2f, %.2f)", 
              STATE_->current_goal_.pose.position.x, STATE_->current_goal_.pose.position.y, STATE_->current_goal_.pose.position.z);
          STATE_->setDecision("d_planning");
        }
        else{
          RCLCPP_DEBUG(this->get_logger(), "Found a plan with its final position: (%.2f, %.2f, %.2f)", 
              plan.back().pose.position.x, plan.back().pose.position.y, plan.back().pose.position.z);
          STATE_->last_valid_plan_ = clock_->now();
          LP_->setPlan(plan);
          LP_->resetRotationReference();
          if (localization_replanning_) {
            localization_replanning_ = false;
            RCLCPP_INFO(get_logger(), "Localization resume plan ready: continuing retained goal.");
          }
          STATE_->setDecision("d_align_heading");  
        }
      }

      if((clock_->now()-STATE_->last_valid_plan_).seconds()>STATE_->planner_patience_){
        RCLCPP_WARN(this->get_logger(), "Time out to find a plan to point (%.2f, %.2f, %.2f)", 
            STATE_->current_goal_.pose.position.x, STATE_->current_goal_.pose.position.y, STATE_->current_goal_.pose.position.z);
        startRecoveryBehaviors("rotate_inplace");
        STATE_->setDecision("d_recovery_waitdone");
        return false;
      }
      return false;
    }
    
    else if(STATE_->isCurrentDecision("d_align_heading")){

      if(LP_->isInitialHeadingAligned()){
        if (stop_after_heading_alignment_) {
          publishZeroVelocity();
          heading_stopped_samples_ = 0;
          heading_last_odom_stamp_ = rclcpp::Time(robot_state_.header.stamp).nanoseconds();
          heading_stop_started_ = std::chrono::steady_clock::now();
          STATE_->setDecision("d_heading_stopping");
          RCLCPP_INFO(get_logger(), "Heading aligned: braking before path tracking.");
        } else {
          STATE_->setDecision("d_controlling");
        }
      }
      else{

        if(STATE_->oscillation_patience_ > 0 && (clock_->now()-STATE_->last_oscillation_reset_).seconds() >= STATE_->oscillation_patience_){
          //@go to recovery
          auto diff = (clock_->now()-STATE_->last_oscillation_reset_).seconds();
          RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, STATE_->getDistance(STATE_->global_pose_, STATE_->oscillation_pose_));
          startRecoveryBehaviors("rotate_inplace");
          STATE_->setDecision("d_recovery_waitdone");  
          return false;
        }
        
        base_trajectory::Trajectory best_traj;
        dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand("differential_drive_rotate_shortest_angle", best_traj);

        if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
          STATE_->last_valid_control_ = clock_->now();
          STATE_->setDecision("d_align_heading");  
          publishVelocity(best_traj);
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
          publishZeroVelocity("感知数据过期或异常");
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::CONFIGURATION_ERROR){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Configuration error, check your yaml and logs.");
          publishZeroVelocity("局部规划器配置错误");
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
          publishZeroVelocity("局部规划器缺少有效里程计或 TF");
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
          //@ this assignment will allow at least one time planning query
          STATE_->last_valid_plan_ = clock_->now();
          publishZeroVelocity("路径裁剪失败，等待重新规划");
          STATE_->setDecision("d_planning");  
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
          //At least implement last_valid_control_ timeout to abort here
          //@ this assignment will allow at least one time planning query
          if((clock_->now() - STATE_->last_valid_control_).seconds() > STATE_->controller_patience_){
            RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
            startRecoveryBehaviors("rotate_inplace");
            STATE_->setDecision("d_recovery_waitdone");
          }
          else{
            STATE_->last_valid_plan_ = clock_->now();
            STATE_->setDecision("d_planning");  
          }
          publishZeroVelocity("全部局部轨迹被拒绝，等待重试");
          return false;
        }

        else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT || PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
          STATE_->last_valid_plan_ = clock_->now();
          STATE_->setDecision("d_planning");
          publishZeroVelocity();
          return false;
        }

        else{
          RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
          publishZeroVelocity();
          return false;
        }
      }

    }

    else if (STATE_->isCurrentDecision("d_heading_stopping")) {
      publishZeroVelocity();
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - heading_stop_started_).count();
      const auto stamp = rclcpp::Time(robot_state_.header.stamp);
      const double age = (clock_->now() - stamp).seconds();
      const auto& velocity = robot_state_.twist.twist;
      const double speed = std::hypot(velocity.linear.x, velocity.linear.y);
      const bool stopped = stamp.nanoseconds() > 0 && age >= 0.0 && age <= 0.5 &&
          std::isfinite(speed) && std::isfinite(velocity.angular.z) &&
          speed <= 0.03 && std::abs(velocity.angular.z) <= 0.05;
      if (!stopped) heading_stopped_samples_ = 0;
      else if (stamp.nanoseconds() > heading_last_odom_stamp_) ++heading_stopped_samples_;
      heading_last_odom_stamp_ = stamp.nanoseconds();
      if (elapsed >= 5.0) {
        RCLCPP_ERROR(get_logger(), "Heading braking timeout: speed=%.3f, yaw_rate=%.3f, odom_age=%.3f; aborting.",
            speed, velocity.angular.z, age);
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        return true;
      }
      if (heading_stopped_samples_ >= 3) {
        STATE_->last_valid_control_ = clock_->now();
        STATE_->setDecision(LP_->isInitialHeadingAligned() ? "d_controlling" : "d_align_heading");
        RCLCPP_INFO(get_logger(), "Heading braking verified on three fresh odometry samples; heading rechecked.");
      }
      return false;
    }

    else if(STATE_->isCurrentDecision("d_align_goal_heading")){
      // Turning can move the body outside the position tolerance. Both conditions
      // must hold at completion, rather than latching an earlier position check.
      if (!LP_->isGoalReached()) {
        publishZeroVelocity();
        STATE_->setDecision("d_controlling");
        return false;
      }
      if(LP_->isGoalHeadingAligned()){
        if (rotation_pulse_.active) {
          publishZeroVelocity();
          return false;  // Confirm position and yaw again after the coupled drift stops.
        }
        RCLCPP_INFO(this->get_logger(), "Goal reach.");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->succeed(result);
        publishZeroVelocity();
        return true;
      }
      else{

        if(STATE_->oscillation_patience_ >0 && (clock_->now()-STATE_->last_oscillation_reset_).seconds() >= STATE_->oscillation_patience_){
          //@go to recovery
          auto diff = (clock_->now()-STATE_->last_oscillation_reset_).seconds();
          RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, STATE_->getDistance(STATE_->global_pose_, STATE_->oscillation_pose_));
          startRecoveryBehaviors("rotate_inplace");
          STATE_->setDecision("d_recovery_waitdone"); 
          return false;
        }
        
        base_trajectory::Trajectory best_traj;
        dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand("differential_drive_rotate_shortest_angle", best_traj);

        if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
          STATE_->last_valid_control_ = clock_->now();
          STATE_->setDecision("d_align_goal_heading");  
          publishVelocity(best_traj);
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
          publishZeroVelocity("感知数据过期或异常");
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::CONFIGURATION_ERROR){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Configuration error, check your yaml and logs.");
          publishZeroVelocity("局部规划器配置错误");
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
          publishZeroVelocity("局部规划器缺少有效里程计或 TF");
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
          //@ this assignment will allow at least one time planning query
          STATE_->last_valid_plan_ = clock_->now();
          publishZeroVelocity("路径裁剪失败，等待重新规划");
          STATE_->setDecision("d_planning");  
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL ||
                PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT || 
                PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
          //At least implement last_valid_control_ timeout to abort here
          //@ this assignment will allow at least one time planning query
          if((clock_->now() - STATE_->last_valid_control_).seconds() > STATE_->controller_patience_){
            RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
            startRecoveryBehaviors("rotate_inplace");
            STATE_->setDecision("d_recovery_waitdone");
          }
          else{
            STATE_->setDecision("d_align_goal_heading");  
          }
          publishZeroVelocity();
          return false;
        }
        else{
          RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
          publishZeroVelocity();
          return false;
        }
      }
    }

    else if(STATE_->isCurrentDecision("d_controlling")){

      //@Check is goal xy tolerance reach
      if(LP_->isGoalReached()){
        publishZeroVelocity();
        if(STATE_->use_position_control_at_goal_){
          RCLCPP_INFO(this->get_logger(), "Goal xy tolerance reach, align the goal with position control.");
          startRecoveryBehaviors("position_control");
          STATE_->setDecision("d_recovery_position_control_waitdone");  
        }
        else{
          STATE_->setDecision("d_align_goal_heading");  
          RCLCPP_INFO(this->get_logger(), "Goal xy tolerance reach, switch to align goal heading state.");
        }
        return false;
      }
      
      //@ update global plan
      if(GPM_->hasPlan()){
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        GPM_->copyPlan(plan);
        LP_->setPlan(plan);
      }
      //@Behavior for oscillation here
      
      if(STATE_->oscillation_patience_ >0 && (clock_->now()-STATE_->last_oscillation_reset_).seconds() >= STATE_->oscillation_patience_){
        //@go to recovery
        auto diff = (clock_->now()-STATE_->last_oscillation_reset_).seconds();
        RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, STATE_->getDistance(STATE_->global_pose_, STATE_->oscillation_pose_));
        startRecoveryBehaviors("rotate_inplace");
        STATE_->setDecision("d_recovery_waitdone");
        return false;
      }

      base_trajectory::Trajectory best_traj;
      dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(STATE_->main_trajectory_generator_, best_traj);

      if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
        STATE_->last_valid_control_ = clock_->now();
        STATE_->setDecision("d_controlling");
        publishVelocity(best_traj);
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
        publishZeroVelocity("感知数据过期或异常");
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::CONFIGURATION_ERROR){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Configuration error, check your yaml and logs.");
        publishZeroVelocity("局部规划器配置错误");
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
        publishZeroVelocity("局部规划器缺少有效里程计或 TF");
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
        //@ this assignment will allow at least one time planning query
        STATE_->last_valid_plan_ = clock_->now();
        publishZeroVelocity("路径裁剪失败，等待重新规划");
        STATE_->setDecision("d_planning");  
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
        //At least implement last_valid_control_ timeout to abort here
        //@ this assignment will allow at least one time planning query
        if((clock_->now() - STATE_->last_valid_control_).seconds() > STATE_->controller_patience_){
          RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
          startRecoveryBehaviors("rotate_inplace");
          STATE_->setDecision("d_recovery_waitdone");
        }
        else{
          STATE_->last_valid_plan_ = clock_->now();
          STATE_->setDecision("d_controlling");
        }
        publishZeroVelocity();
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
        STATE_->last_valid_plan_ = clock_->now();
        publishZeroVelocity("局部路径阻塞，等待重新规划");
        STATE_->setDecision("d_planning"); 
        RCLCPP_WARN(this->get_logger(), "Path conflits, but no need to wait.");
       	return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT){
        STATE_->waiting_time_ = clock_->now();
        STATE_->setDecision("d_waiting");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Path conflits, switch to waiting state.");
       	return false;
      }

      else{
        RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
        publishZeroVelocity();
        return false;
      }

    }

    else if(STATE_->isCurrentDecision("d_recovery_position_control_waitdone")){
      
      if(is_recoverying_){
        return false;
      }
      
      if(is_recoverying_succeed_){
        //we go to planning and we also need to count second recovery then abort
        RCLCPP_INFO(this->get_logger(), "Position control succeed, go to align goal heading state.");
        STATE_->last_valid_plan_ = clock_->now();
        STATE_->setDecision("d_align_goal_heading");  
        return false;  
      }
      else{
        //we may abort or go to another recovery
        RCLCPP_ERROR(this->get_logger(), "The potential collision has been detected when doing recovery - position control.");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;  
      }
      
    }

    else if(STATE_->isCurrentDecision("d_recovery_waitdone")){
      if (!enable_rotate_recovery_) {
        RCLCPP_ERROR(get_logger(), "Rotate recovery disabled: aborting navigation after planner/controller failure.");
        publishZeroVelocity();
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        return true;
      }
      
      if(is_recoverying_){
        return false;
      }
        

      if(STATE_->no_plan_recovery_count_>=STATE_->no_plan_retry_num_){
        RCLCPP_ERROR(this->get_logger(), "No global plan has been found even we try recovery %d times", STATE_->no_plan_recovery_count_);
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;        
      }
      
      if(is_recoverying_succeed_){
        //we go to planning and we also need to count second recovery then abort
        RCLCPP_INFO(this->get_logger(), "Recovery succeed, back to planning state.");
        STATE_->no_plan_recovery_count_++;
        STATE_->last_valid_plan_ = clock_->now();
        STATE_->setDecision("d_planning");
        return false;  
      }
      else{
        //we may abort or go to another recovery
        RCLCPP_ERROR(this->get_logger(), "The potential collision has been detected when doing recovery.");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;  
      }
      
    }

    else if(STATE_->isCurrentDecision("d_waiting")){
      
      //if continue conflict over 10s,to recalculate the path
      if((clock_->now()-STATE_->waiting_time_).seconds() >= STATE_->waiting_patience_){ 
       	STATE_->last_valid_plan_ = clock_->now();
        STATE_->setDecision("d_planning");
        RCLCPP_WARN(this->get_logger(), "waiting time over %.2f,change to d_planning", STATE_->waiting_patience_);
        return false;
      }

      //@ update global plan
      if(GPM_->hasPlan()){
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        GPM_->copyPlan(plan);
        LP_->setPlan(plan);
      }
      base_trajectory::Trajectory best_traj;
      dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(STATE_->main_trajectory_generator_, best_traj);

      if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
        STATE_->last_valid_control_ = clock_->now();
        STATE_->setDecision("d_controlling");
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
        publishZeroVelocity("感知数据过期或异常");
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::CONFIGURATION_ERROR){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Configuration error, check your yaml and logs.");
        publishZeroVelocity("局部规划器配置错误");
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
        publishZeroVelocity("局部规划器缺少有效里程计或 TF");
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
        STATE_->last_valid_plan_ = clock_->now();
        publishZeroVelocity("路径裁剪失败，等待重新规划");
        STATE_->setDecision("d_planning");  
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
        //At least implement last_valid_control_ timeout to abort here
        //@ this assignment will allow at least one time planning query
        if((clock_->now() - STATE_->last_valid_control_).seconds() > STATE_->controller_patience_){
          RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
          startRecoveryBehaviors("rotate_inplace");
          STATE_->setDecision("d_recovery_waitdone");
        }
        else{
          STATE_->last_valid_plan_ = clock_->now();
          STATE_->setDecision("d_planning");  
        }
        publishZeroVelocity();
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT || PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
	      STATE_->setDecision("d_waiting");
        publishZeroVelocity();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Path conflits in waiting state, keep waiting.");
	      return false;
      }

      else{
        RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
        publishZeroVelocity();
        return false;
      }
    }

  return false;
}

void P2PMoveBase::startRecoveryBehaviors(std::string behavior_name){
  if (behavior_name == "rotate_inplace" && !enable_rotate_recovery_) {
    publishZeroVelocity();
    is_recoverying_ = false;
    is_recoverying_succeed_ = false;
    RCLCPP_WARN(get_logger(), "Rotate recovery request suppressed by enable_rotate_recovery=false.");
    return;
  }

  auto goal_msg = dddmr_sys_core::action::RecoveryBehaviors::Goal();
  goal_msg.behavior_name = behavior_name;
  goal_msg.target_pose = STATE_->current_goal_;

  auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::RecoveryBehaviors>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&P2PMoveBase::recovery_behaviors_client_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&P2PMoveBase::recovery_behaviors_client_result_callback, this, std::placeholders::_1);
  
  is_recoverying_ = true;
  recovery_behaviors_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void P2PMoveBase::recovery_behaviors_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    is_recoverying_ = false;
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by recovery behaviors server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by recovery behaviors server, waiting for result");
  }
}

void P2PMoveBase::recovery_behaviors_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::WrappedResult & result)
{
  is_recoverying_succeed_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      is_recoverying_succeed_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Unknown result code");
      break;
  }
  
  is_recoverying_ = false;
}

}//end of name space
