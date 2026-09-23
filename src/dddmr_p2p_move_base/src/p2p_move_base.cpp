#include <dddmr_sys_core/motion_timestamp.h>
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
  continuous_path_tracking_ = declare_parameter<bool>("continuous_path_tracking", false);
  heading_trajectory_generator_ = declare_parameter<std::string>(
    "heading_trajectory_generator", "differential_drive_rotate_shortest_angle");
  if (heading_trajectory_generator_.empty())
    throw std::invalid_argument("heading_trajectory_generator cannot be empty");
  if (continuous_path_tracking_ && (!rotation_angle_feedback_ ||
      rotation_predict_duration_ || rotation_pulse_duration_ > 0.0))
    throw std::invalid_argument("Continuous tracking requires angle feedback and disabled fixed/predicted rotation durations");
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
  declare_parameter<bool>("relocalization_only", false);
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
  if (continuous_path_tracking_ &&
      LP_->get_parameter("tracking_trajectory_generator").as_string() != STATE_->main_trajectory_generator_)
    throw std::invalid_argument("Continuous tracking requires local_planner.tracking_trajectory_generator to match main_trajectory_generator");
  
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

  // Separate endpoint, same task ownership: test and navigation cannot run together.
  rotation_test_server_=rclcpp_action::create_server<dddmr_sys_core::action::PToPMoveBase>(
    this,"/rotation_test",
    [this](const rclcpp_action::GoalUUID&,std::shared_ptr<const dddmr_sys_core::action::PToPMoveBase::Goal> goal) {
      if (!std::isfinite(goal->target_value) || std::abs(goal->target_value)<0.0174533 ||
          std::abs(goal->target_value)>2.96706 || !continuous_path_tracking_ || task_running_.exchange(true))
        return rclcpp_action::GoalResponse::REJECT;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },std::bind(&P2PMoveBase::handle_cancel,this,std::placeholders::_1),
    [this](auto handle){std::thread([this,handle]{executeRotationTest(handle);}).detach();},
    rcl_action_server_get_default_options(),action_server_group_);
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
  // Continuous tracking may steer while translating. Dedicated initial/final
  // alignment retains the rotation feedback and settling gates.
  const bool continuous_tracking = continuous_path_tracking_ && STATE_->isCurrentDecision("d_controlling");
  if (!continuous_tracking && (rotation_angle_feedback_ || rotation_predict_duration_ || rotation_pulse_duration_ > 0.0)) {
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
      if (!dddmr_sys_core::motionTimestampFresh(stamp.nanoseconds(), age) ||
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
    const bool moving = std::abs(cmd_traj.xv_) > 1e-6 || std::abs(cmd_traj.yv_) > 1e-6 || std::abs(cmd_traj.thetav_) > 1e-6;
    const int action = continuous_tracking && moving ? 5 : std::abs(cmd_traj.thetav_) > 1e-6 ? 1 :
      std::abs(cmd_traj.yv_) > 1e-6 ? 2 :
      cmd_traj.xv_ > 1e-6 ? 3 : cmd_traj.xv_ < -1e-6 ? 4 : 0;
    if (action == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
        "停车原因：局部规划器选中零速度轨迹；查看 trajectory_generators 的停车诊断; state=%s",
        STATE_->getCurrentDecision().c_str());
    }
    if (action != 0 && action != last_navigation_action_) {
      last_navigation_action_ = action;
      const char* label = action == 5 ? "连续三轴路径跟踪中…" : action == 1 ? "旋转对方向中…" :
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

// 一个已接受的导航 Action 对应一次 executeCb，由 handle_accepted 创建的线程执行。
// 管理整次任务的生命周期：校验目标 -> 选择定位模式 -> 初始化状态/目标 -> 周期决策。
// executeCycle 负责单轮状态机，本函数负责循环节拍、取消检查、反馈和结束时的清理。
// 它不是 ROS 定时器回调；无导航任务时不会运行这个控制循环。
void P2PMoveBase::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
  // 作用域退出守卫：包括下方提前 return，都会释放任务占用标志，允许后续目标进入。
  auto finish_task = std::shared_ptr<void>(nullptr, [this](void*) { task_running_ = false; });
  auto move_base_goal = goal_handle->get_goal();

  // 目标姿态必须具有有效四元数；失败时停车并报告 Action 中止，不进入控制循环。
  if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
    RCLCPP_WARN(this->get_logger(),"Aborting on goal because it was sent with an invalid quaternion");
    goal_handle->abort(result);
    publishZeroVelocity();
    return;
  }

  // 定位模式按本次任务确定：持续 odom-only 策略优先于 use_mcl_during_navigation。
  // persistent_odom=true：显式重定位后保持冻结 map->odom，任务结束也不自动恢复匹配。
  // 否则由 use_mcl_during_navigation 决定任务中持续匹配，还是临时切到纯里程计。
  const bool persistent_odom = get_parameter("relocalization_only").as_bool();
  task_use_mcl_ = !persistent_odom && get_parameter("use_mcl_during_navigation").as_bool();
  // 仅“临时纯里程计”模式在作用域退出时停车并请求恢复 MCL；成功、取消、失败均覆盖。
  // 持续纯里程计和持续 MCL 模式不在此切换；服务失败只记录错误，不代表恢复成功。
  auto restore_mcl = std::shared_ptr<void>(nullptr, [this, odom_only = !task_use_mcl_ && !persistent_odom](void*) {
    if (odom_only) {
      publishZeroVelocity();
      if (!setOdomOnly(false))
        RCLCPP_ERROR(get_logger(), "MCL restore failed: reload navigation before the next task.");
    }
  });
  if (!task_use_mcl_) {
    // 临时纯里程计要求初始定位仍有效；持续纯里程计跳过这里的有效期判断。
    // 两者均要求 setOdomOnly(true) 服务成功，否则拒绝开始任务。
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if ((!persistent_odom && (localization_timeout_ <= 0 || now >= localization_valid_until_.load())) || !setOdomOnly(true)) {
      publishZeroVelocity();
      RCLCPP_ERROR(get_logger(), "Navigation mode rejected: valid initial localization and MCL mode service required.");
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(get_logger(), "Odometry-only navigation: map-to-odom frozen; MCL policy controls when matching resumes.");
  }
  // 外层任务循环的目标频率；耗时超过周期时，实际调用频率可能低于配置值。
  rclcpp::Rate r(STATE_->controller_frequency_);

  // 重置本次任务状态、计时及振荡参考位姿，避免沿用上次任务的超时/运动状态。
  // 有效运动进展看门狗还会在首次 publishVelocity 时重新起计。
  STATE_->initialParams(LP_->getGlobalPose(), clock_->now());
  progress_control_started_ = false;
  last_navigation_action_ = 0;
  rotation_pulse_.reset();
  LP_->resetRotationReference();
  localization_paused_ = false;
  localization_replanning_ = false;
  obstacle_replan_.clear();
  // 保存目标并恢复全局规划管理器；路径是否可用由后续 executeCycle 检查，
  // resume() 本身不表示已有可执行路径，也不在这里直接发布运动命令。
  STATE_->current_goal_ = move_base_goal->target_pose;
  GPM_->setGoal(STATE_->current_goal_);
  GPM_->resume();

  while(rclcpp::ok()){

    // 每轮决策前先检查 Action 生命周期；已失活时取消恢复动作、停车并结束规划。
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

    // 客户端请求取消：取消独立恢复 Action，报告 canceled，停车并退出任务线程。
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

    // 已接受目标且初始化通过后，每个控制周期推进一次任务状态机。
    // 非活动/取消目标已在上方处理；false 表示下周期继续，true 表示任务已成功或中止。
    // 调用频率由上方 rclcpp::Rate(controller_frequency_) 和下方 r.sleep() 控制，
    // 不是传感器回调，也不是每次全局重规划才调用一次。
    bool done = executeCycle(goal_handle);
    
    // 将本轮保存的全局位姿和状态转移作为 Action 反馈；不是底盘执行成功的回执。
    auto feedback = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Feedback>();
    feedback->base_position = STATE_->global_pose_;
    feedback->last_decision = STATE_->getLastDecision();
    feedback->current_decision = STATE_->getCurrentDecision();
    goal_handle->publish_feedback(feedback);

    // true 只表示任务已结束，成功或中止结果已由 executeCycle 报告；这里不重复报告。
    if(done){
      GPM_->stop();
      return;
    }
    
    // 等待下一控制周期；等待路径、定位恢复等非运动状态也保持这个循环。
    r.sleep();

    //if(STATE_->isCurrentDecision("d_controlling") && r.cycleTime() > ros::Duration(1 / STATE_->controller_frequency_))
    //  ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", STATE_->controller_frequency_, r.cycleTime().toSec());
  }
  // rclcpp::ok() 变为 false（例如 ROS 关闭）时退出循环并停止规划管理器。
  GPM_->stop();
}

// 执行导航任务的一轮决策，而不是在本函数内一次走完整条路径。
// 调用链：Action 接受目标 -> handle_accepted() 创建线程 -> executeCb() 循环调用。
// 前置条件：executeCb 已校验目标、设置定位模式、初始化 STATE_ 并提交目标给 GPM_。
// 每轮先执行公共安全检查，再按 STATE_ 当前状态执行一个分支；setDecision() 只改变
// 后续周期的分支，不会立即跳入同一轮的另一个 else-if。等待/恢复期间也会周期调用。
// 常规流程：initial -> planning -> planning_waitdone -> validate_path -> align_heading
//           -> [heading_stopping] -> controlling -> align_goal_heading -> 成功。
// 异常分支可以停车等待、重新规划、进入恢复或中止；并非每轮都会下发运动速度。
// 返回 true：已通过 goal_handle 报告成功/中止，executeCb 应停止规划管理器并退出。
// 返回 false：任务尚未结束（包括停车等待），executeCb 发布反馈后等待下一个周期。
bool P2PMoveBase::executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle){
    // 公共门控 1：持续障碍重规划超时则结束任务，不能靠重复请求无限延后截止时间。
    const double obstacle_now = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (obstacle_replan_.expired(obstacle_now, STATE_->controller_patience_)) {
      publishZeroVelocity("障碍阻塞持续超时，停止任务");
      RCLCPP_ERROR(get_logger(), "Obstacle replanning timeout: no executable path; navigation aborted.");
      goal_handle->abort(std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>());
      return true;
    }

    // 公共门控 2：仅在本任务使用持续 MCL 时检查定位有效期；失效后保留目标停车，
    // 待定位稳定且恢复动作结束，再从当前位置重规划。冻结 map->odom 模式不走此门控。
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
    // 公共门控 3：规划服务/响应失效与“本次障碍快照无路”不同：前者中止，后者停车重试。
    if (GPM_->planningUnsafe()) {
      publishZeroVelocity();
      recovery_behaviors_client_ptr_->async_cancel_all_goals();
      RCLCPP_ERROR(get_logger(), "Planning unavailable/stale: navigation aborted, cached local path disabled.");
      auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
      goal_handle->abort(result);
      return true;
    }
    if (GPM_->consumeNoPathEvent()) {
      publishZeroVelocity("当前障碍快照暂无全局路径，保持停车并重试");
      // Start a bounded retry episode. Further empty results do not reset this
      // deadline; planner_patience still decides when recovery is necessary.
      STATE_->last_valid_plan_ = clock_->now();
      STATE_->setDecision("d_planning");
      RCLCPP_WARN(get_logger(),
        "Global planner returned no path for the current obstacle snapshot; "
        "stopped and retrying for up to %.1f s.", STATE_->planner_patience_);
      return false;
    }

    // 同步本轮位姿/里程计；若旋转脉冲仍处于制动或停稳确认阶段，先处理它，
    // 暂不进入普通状态分支，避免尚未停稳就重新下发下一段运动。
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

    // 更新无进展/振荡看门狗：水平位移或朝向变化足够大才重置计时。
    // 四足步态的 Z 向起伏不能当成沿地面的有效进展。
    const double progress_xy = std::hypot(
        STATE_->global_pose_.transform.translation.x - STATE_->oscillation_pose_.transform.translation.x,
        STATE_->global_pose_.transform.translation.y - STATE_->oscillation_pose_.transform.translation.y);
    if(progress_xy >= STATE_->oscillation_distance_ ||
          STATE_->getAngle(STATE_->global_pose_, STATE_->oscillation_pose_) >= STATE_->oscillation_angle_)
    {
      STATE_->oscillation_pose_ = STATE_->global_pose_;
      STATE_->last_oscillation_reset_ = clock_->now();
    }


    // 初始化状态只切换到规划状态，实际查询留到下一控制周期。
    if(STATE_->isCurrentDecision("d_initial")){
      STATE_->setDecision("d_planning");
    }

    // 请求规划管理器查询路径，然后进入异步结果等待状态；这里不执行局部跟踪。
    else if(STATE_->isCurrentDecision("d_planning")){
      GPM_->queryThread();
      STATE_->setDecision("d_planning_waitdone");
      return false;
    }

    // 检查全局路径结果：至少三个点才交给局部规划器，并进入通行检查。
    // 普通短路径回到规划重试；定位恢复后的短路径直接中止；规划超时则请求恢复。
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
        if(plan.size()<3){
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
          STATE_->setDecision("d_validate_path");
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
    
    // 有全局路径不等于车体可通行：保持零速度，检查路径/感知/TF，
    // 由障碍重规划门控决定允许对齐、重新查询，还是继续等待。
    else if(STATE_->isCurrentDecision("d_validate_path")) {
      publishZeroVelocity("新路径等待通行检查，保持停车");
      const auto status=LP_->checkPathBeforeAlignment();
      const bool clear=status==dddmr_sys_core::TRAJECTORY_FOUND;
      const bool checked=clear || status==dddmr_sys_core::PATH_BLOCKED_REPLANNING ||
                         status==dddmr_sys_core::PRUNE_PLAN_FAIL;
      const auto admission=obstacle_replan_.admit(obstacle_now,checked,clear);
      if (admission==ObstacleReplan::Admission::Align) {
        LP_->resetRotationReference();
        STATE_->setDecision("d_align_heading");
        RCLCPP_INFO(get_logger(), "新路径通行检查通过，允许进入朝向对齐");
      } else if (admission==ObstacleReplan::Admission::Replan) {
        GPM_->resume(true);
        STATE_->last_valid_plan_=clock_->now();
        STATE_->setDecision("d_planning");
        RCLCPP_WARN(get_logger(), "新路径仍阻塞或无效：保持停车重新规划，不进入旋转");
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(),*clock_,1000,
          "新路径尚未通过检查：保持停车，等待感知/TF 或重规划间隔; status=%d",
          static_cast<int>(status));
      }
      return false;
    }

    // 起步/重规划后的朝向对齐：对准参考路径方向，不是终点姿态。
    // 未对齐时显式选用 shortest_angle 旋转生成器；对齐后按配置先停稳或直接跟踪。
    else if(STATE_->isCurrentDecision("d_align_heading")){
      // Recheck while aligning too: a newly observed obstruction must not
      // permit repeated turns simply because an earlier snapshot was clear.
      if (LP_->checkPathBeforeAlignment()!=dddmr_sys_core::TRAJECTORY_FOUND) {
        publishZeroVelocity("对齐期间路径不再可通行，停车重新检查");
        STATE_->setDecision("d_validate_path");
        return false;
      }


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
        dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(heading_trajectory_generator_, best_traj);

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

    // 起步旋转后的制动确认：连续三个不同时间戳的有效低速里程计样本才算停稳。
    // 停稳后复查方向；仍对齐则跟踪，否则重新对齐。超过五秒未确认则中止。
    else if (STATE_->isCurrentDecision("d_heading_stopping")) {
      publishZeroVelocity();
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - heading_stop_started_).count();
      const auto stamp = rclcpp::Time(robot_state_.header.stamp);
      const double age = (clock_->now() - stamp).seconds();
      const auto& velocity = robot_state_.twist.twist;
      const double speed = std::hypot(velocity.linear.x, velocity.linear.y);
      const bool stopped = dddmr_sys_core::motionTimestampFresh(stamp.nanoseconds(), age) &&
          std::isfinite(speed) && std::isfinite(velocity.angular.z) &&
          speed <= 0.03 && std::abs(velocity.angular.z) <= 0.05;
      if (!stopped || stamp.nanoseconds() < heading_last_odom_stamp_) heading_stopped_samples_ = 0;
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

    // 终点朝向对齐：由 controlling 的 isGoalReached() 成立，或终点位置控制成功进入。
    // isGoalReached 比较当前 XY 与完整全局路径末点的距离，并非看状态名就认定到达。
    // 位置仍达标但朝向未达标时，下方调用 shortest_angle；两者达标且无活动旋转脉冲
    // 才报告成功。旋转导致位置超差时回到 controlling，重新靠近终点。
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
        dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(heading_trajectory_generator_, best_traj);

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

    // 常规路径跟踪：先判断终点位置，再更新参考路径并调用配置的主轨迹生成器。
    // 正常结果经 publishVelocity 下发；障碍触发停车/强制重规划，不受常规查询周期限制。
    // 感知/TF/配置异常停车等待；裁剪失败回到规划；持续无可用轨迹则请求恢复。
    else if(STATE_->isCurrentDecision("d_controlling")){

      //@Check is goal xy tolerance reach
      if(LP_->isGoalReached()){
        publishZeroVelocity();
        // Continuous steering has no active rotation pulse. Explicitly enter
        // settling so goal success is checked again after three fresh stopped
        // samples, even when the final heading is already within tolerance.
        if (continuous_path_tracking_) rotation_pulse_.stop(steady_now*1e-9);
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
      if (PS == dddmr_sys_core::PATH_BLOCKED_REPLANNING || PS == dddmr_sys_core::PATH_BLOCKED_WAIT) {
        publishZeroVelocity("提前检测到障碍，制动并重规划");
        if (obstacle_replan_.blocked(obstacle_now)) {
          // Invalidate cached/in-flight results before requesting the retained goal.
          GPM_->resume(true);
          STATE_->last_valid_plan_ = clock_->now();
          STATE_->setDecision("d_planning");
          RCLCPP_WARN(get_logger(), "Obstacle replanning requested: cached plan discarded; waiting for a fresh result.");
        }
        return false;
      }
      if (PS == dddmr_sys_core::TRAJECTORY_FOUND &&
          std::hypot(best_traj.xv_,best_traj.yv_)>1e-6)
        obstacle_replan_.clear();


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


      else{
        RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
        publishZeroVelocity();
        return false;
      }

    }

    // 可选的终点位置控制由独立恢复 Action 执行：这里只轮询结果。
    // 成功后仍需终点朝向对齐；失败则中止，而不是直接宣布导航完成。
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

    // 普通旋转恢复：禁用恢复则直接中止；启用时等待恢复结果，
    // 在重试次数允许范围内成功后重新规划，失败或次数耗尽则中止。
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

    // 等待分支：等待超时则重规划；期间重新评估局部轨迹。
    // 找到轨迹时只切回 controlling，实际速度在后续跟踪周期发布。
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
