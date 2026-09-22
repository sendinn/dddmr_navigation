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
#include <p2p_move_base/p2p_global_plan_manager.h>
namespace p2p_move_base
{
P2PGlobalPlanManager::P2PGlobalPlanManager(std::string name) : Node(name), name_(name), got_first_goal_(false){
  clock_ = this->get_clock();
}

P2PGlobalPlanManager::~P2PGlobalPlanManager(){
  tf2Buffer_.reset();
  tfl_.reset();
}

void P2PGlobalPlanManager::initial(){
  result_timeout_ = declare_parameter<double>("plan_result_timeout", 2.0);
  if (!std::isfinite(result_timeout_) || result_timeout_ <= 0.0)
    throw std::invalid_argument("plan_result_timeout must be positive and finite");

  this->declare_parameter("global_planner_action_name", rclcpp::ParameterValue("get_plan"));
  this->get_parameter("global_planner_action_name", global_planner_action_name_);
  RCLCPP_INFO(this->get_logger(), "P2P global plan manager uses \033[1;32m%s\033[0m service to query global plan.", global_planner_action_name_.c_str());

  this->declare_parameter("global_plan_query_frequency", rclcpp::ParameterValue(5.0));
  this->get_parameter("global_plan_query_frequency", global_plan_query_frequency_);
  RCLCPP_INFO(this->get_logger(), "global_plan_query_frequency: %.2f", global_plan_query_frequency_);

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
  
  global_planner_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  global_planner_client_ptr_ = rclcpp_action::create_client<dddmr_sys_core::action::GetPlan>(
      this,
      global_planner_action_name_, global_planner_client_group_);
  
  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if(global_plan_query_frequency_>0.0){
    auto loop_time = std::chrono::milliseconds(int(1000/global_plan_query_frequency_));
    loop_timer_ = this->create_wall_timer(loop_time, std::bind(&P2PGlobalPlanManager::queryThread, this), timer_group_);
    stop();
  }
  else{
    auto loop_time = std::chrono::milliseconds(1000000000);
    loop_timer_ = this->create_wall_timer(loop_time, std::bind(&P2PGlobalPlanManager::queryThread, this), timer_group_);
    stop();
  }

}

void P2PGlobalPlanManager::resume(bool force_full_replan){
  std::unique_lock<std::mutex> lock(access_);
  // DWA normally treats repeated requests for the same stamped goal as a
  // cache query and only repairs the short look-ahead prefix.  Once the local
  // planner has rejected that route because of an obstacle, keeping the same
  // stamp can therefore keep attaching the blocked reference tail forever.
  // A fresh stamp deliberately makes DWA rebuild the whole route from the
  // current robot pose to the retained final goal.
  if (force_full_replan)
    goal_.header.stamp = clock_->now();
  ++generation_;
  failed_ = false;
  no_path_active_ = false;
  no_path_event_ = false;
  last_result_time_ = std::chrono::steady_clock::now();
  global_path_.poses.clear();
  is_planning_ = false;
  loop_timer_->reset();
  if (force_full_replan)
    RCLCPP_WARN(this->get_logger(),
      "Global plan manager requests a full route rebuild for the retained goal");
  else
    RCLCPP_INFO(this->get_logger(), "Global plan manager is resumed");
}

void P2PGlobalPlanManager::pause(){
  std::unique_lock<std::mutex> lock(access_);
  ++generation_;  // Discard outstanding results; no stop request can race the restart.
  loop_timer_->cancel();
  global_path_.poses.clear();
  is_planning_ = false;
  no_path_active_ = false;
  no_path_event_ = false;
}

void P2PGlobalPlanManager::stop(){
  std::unique_lock<std::mutex> lock(access_);
  ++generation_;
  loop_timer_->cancel();
  
  if(got_first_goal_){
    auto goal_msg = dddmr_sys_core::action::GetPlan::Goal();
    goal_msg.activate_threading = false;
    auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::GetPlan>::SendGoalOptions();
    // Stop acknowledgements contain no path and must not affect a later goal.
    global_planner_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    got_first_goal_ = false;
  }

  RCLCPP_INFO(this->get_logger(), "Global plan manager is stopped");
}

void P2PGlobalPlanManager::queryThread(){

  std::unique_lock<std::mutex> lock(access_);
  
  if(is_planning_)
    return;
  
  if(!got_first_goal_)
    return;

  auto goal_msg = dddmr_sys_core::action::GetPlan::Goal();
  goal_msg.goal = goal_;
  goal_msg.activate_threading = true;

  auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::GetPlan>::SendGoalOptions();
  const auto generation = generation_;
  send_goal_options.goal_response_callback = [this, generation](const auto& handle) {
    std::unique_lock<std::mutex> lock(access_);
    if (generation != generation_) return;
    if (!handle) { failed_ = true; is_planning_ = false; global_path_.poses.clear(); }
    global_planner_client_goal_response_callback(handle);
  };
  send_goal_options.result_callback = [this, generation](const auto& result) {
    std::unique_lock<std::mutex> lock(access_);
    if (generation != generation_) return;
    global_planner_client_result_callback(result);
  };
  
  is_planning_ = true;
  global_planner_client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void P2PGlobalPlanManager::global_planner_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::GetPlan>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    if(global_plan_query_frequency_>2)
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *clock_, 5000, "Goal was rejected by: %s", global_planner_action_name_.c_str());
    else
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by: %s", global_planner_action_name_.c_str());
  } else {
    if(global_plan_query_frequency_>2)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "Goal accepted by: %s, waiting for result", global_planner_action_name_.c_str());
    else
      RCLCPP_INFO(this->get_logger(), "Goal accepted by: %s, waiting for result", global_planner_action_name_.c_str()); 
  }
}

void P2PGlobalPlanManager::global_planner_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::GetPlan>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      //RCLCPP_INFO(this->get_logger(), "Global Planner ---> %s: Global plan is found", global_planner_action_name_.c_str());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 2000,
        "Global Planner ---> %s: no path in current obstacle snapshot; keep stopped and retry",
        global_planner_action_name_.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Global Planner ---> %s: Goal was canceled", global_planner_action_name_.c_str());
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Global Planner ---> %s: Unknown result code", global_planner_action_name_.c_str());
      break;
  }
  last_result_time_ = std::chrono::steady_clock::now();
  const bool valid_no_path_response =
    result.code == rclcpp_action::ResultCode::ABORTED ||
    (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      (!result.result || result.result->path.poses.empty()));
  if (valid_no_path_response) {
    // An empty collision-aware plan is a valid, retriable planner response.
    // Treating it as a transport failure used to abort navigation after one
    // transient lidar frame, bypassing planner_patience entirely.
    failed_ = false;
    if (!no_path_active_) no_path_event_ = true;
    no_path_active_ = true;
    global_path_.poses.clear();
  } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result) {
    failed_ = false;
    no_path_active_ = false;
    no_path_event_ = false;
    global_path_ = result.result->path;
  } else {
    failed_ = true;
    global_path_.poses.clear();
  }
  is_planning_ = false;
}

void P2PGlobalPlanManager::setGoal(const geometry_msgs::msg::PoseStamped& goal){
  std::unique_lock<std::mutex> lock(access_);
  goal_ = goal;
  got_first_goal_ = true;
}

bool P2PGlobalPlanManager::hasPlan(){
  std::unique_lock<std::mutex> lock(access_);
  if(!is_planning_ && !global_path_.poses.empty())
    return true;
  return false;
}

bool P2PGlobalPlanManager::consumeNoPathEvent(){
  std::unique_lock<std::mutex> lock(access_);
  const bool event = no_path_event_;
  no_path_event_ = false;
  return event;
}

bool P2PGlobalPlanManager::planningUnsafe(){
  std::unique_lock<std::mutex> lock(access_);
  const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_result_time_).count();
  if (failed_ || age > result_timeout_ ||
      !global_planner_client_ptr_->action_server_is_ready()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *clock_, 2000,
        "Planning watchdog: failed=%d, result_age=%.3f s, timeout=%.3f s, server_ready=%d; discard cached path",
        failed_, age, result_timeout_, global_planner_client_ptr_->action_server_is_ready());
    failed_ = true;
    global_path_.poses.clear();
    return true;
  }
  return false;
}

void P2PGlobalPlanManager::copyPlan(std::vector<geometry_msgs::msg::PoseStamped>& plan){
  std::unique_lock<std::mutex> lock(access_);
  for(int i=0;i<global_path_.poses.size();i++){
    plan.push_back(global_path_.poses[i]);
  }
}

}
