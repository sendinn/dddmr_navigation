#include <trajectory_generators/braking_rollout.h>
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
#include <trajectory_generators/omni_simple_trajectory_generator_theory.h>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::OmniSimpleTrajectoryGeneratorTheory, trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{

OmniSimpleTrajectoryGeneratorTheory::OmniSimpleTrajectoryGeneratorTheory(){
  return;
}

void OmniSimpleTrajectoryGeneratorTheory::configurateActuatorType(){
  actuator_type_ = dddmr_sys_core::ActuatorType::MOTOR;
}


void OmniSimpleTrajectoryGeneratorTheory::onInitialize(){

  single_axis_tracking_ = node_->declare_parameter<bool>(name_ + ".single_axis_tracking", false);
  axis_yaw_enter_ = node_->declare_parameter<double>(name_ + ".axis_yaw_enter", 0.1745329252);
  axis_yaw_exit_ = node_->declare_parameter<double>(name_ + ".axis_yaw_exit", 0.0872664626);
  axis_lateral_enter_ = node_->declare_parameter<double>(name_ + ".axis_lateral_enter", 0.10);
  axis_lateral_exit_ = node_->declare_parameter<double>(name_ + ".axis_lateral_exit", 0.05);
  axis_obstacle_avoidance_ = node_->declare_parameter<bool>(
    name_ + ".axis_obstacle_avoidance", false);
  avoidance_clear_cycles_ = node_->declare_parameter<int>(
    name_ + ".axis_avoidance_clear_cycles", 3);
  avoidance_switch_cycles_ = node_->declare_parameter<int>(
    name_ + ".axis_avoidance_switch_cycles", 3);
  avoidance_timeout_ = node_->declare_parameter<double>(
    name_ + ".axis_avoidance_timeout", 8.0);
  avoidance_retry_delay_ = node_->declare_parameter<double>(
    name_ + ".axis_avoidance_retry_delay", 2.0);
  if (!std::isfinite(axis_yaw_enter_) || !std::isfinite(axis_yaw_exit_) ||
      !std::isfinite(axis_lateral_enter_) || !std::isfinite(axis_lateral_exit_) ||
      axis_yaw_exit_ <= 0 || axis_yaw_enter_ < axis_yaw_exit_ || axis_yaw_enter_ >= 1.5707963268 ||
      axis_lateral_exit_ <= 0 || axis_lateral_enter_ <= axis_lateral_exit_)
    throw std::invalid_argument("Invalid single-axis tracking thresholds");
  if (avoidance_clear_cycles_ < 1 || avoidance_clear_cycles_ > 100 ||
      avoidance_switch_cycles_ < 1 || avoidance_switch_cycles_ > 100 ||
      !std::isfinite(avoidance_timeout_) || avoidance_timeout_ < 0.5 || avoidance_timeout_ > 60.0 ||
      !std::isfinite(avoidance_retry_delay_) || avoidance_retry_delay_ < 0.0 ||
      avoidance_retry_delay_ > 30.0)
    throw std::invalid_argument("Invalid single-axis obstacle avoidance parameters");
  sample_braking_commands_ = node_->declare_parameter<bool>(name_ + ".sample_braking_commands", false);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sample_braking_commands: %s",
      sample_braking_commands_ ? "true" : "false");

  braking_reaction_time_ = node_->declare_parameter<double>(name_ + ".braking_reaction_time", 0.0);
  if (!std::isfinite(braking_reaction_time_) || braking_reaction_time_ < 0 || braking_reaction_time_ > 2)
    throw std::invalid_argument("braking_reaction_time must be in [0,2] seconds");

  //@initialize trajectory generator
  limits_ = std::make_shared<trajectory_generators::OmniTrajectoryGeneratorLimits>();
  
  node_->declare_parameter(name_ + ".min_vel_x", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_x", limits_->min_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_x: %.2f", limits_->min_vel_x);

  node_->declare_parameter(name_ + ".max_vel_x", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_x", limits_->max_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_x: %.2f", limits_->max_vel_x);

  node_->declare_parameter(name_ + ".min_vel_y", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_y", limits_->min_vel_y);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_y: %.2f", limits_->min_vel_y);

  node_->declare_parameter(name_ + ".max_vel_y", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_y", limits_->max_vel_y);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_y: %.2f", limits_->max_vel_y);

  node_->declare_parameter(name_ + ".min_vel_trans", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_trans", limits_->min_vel_trans);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_trans: %.2f", limits_->min_vel_trans);

  node_->declare_parameter(name_ + ".max_vel_trans", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_trans", limits_->max_vel_trans);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_trans: %.2f", limits_->max_vel_trans);

  node_->declare_parameter(name_ + ".min_vel_theta", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".min_vel_theta", limits_->min_vel_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_theta: %.2f", limits_->min_vel_theta);

  node_->declare_parameter(name_ + ".max_vel_theta", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_theta", limits_->max_vel_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_theta: %.2f", limits_->max_vel_theta);

  node_->declare_parameter(name_ + ".acc_lim_x", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".acc_lim_x", limits_->acc_lim_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_x: %.2f", limits_->acc_lim_x);

  node_->declare_parameter(name_ + ".acc_lim_y", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".acc_lim_y", limits_->acc_lim_y);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_y: %.2f", limits_->acc_lim_y);

  node_->declare_parameter(name_ + ".acc_lim_theta", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".acc_lim_theta", limits_->acc_lim_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_theta: %.2f", limits_->acc_lim_theta);

  node_->declare_parameter(name_ + ".prune_forward", rclcpp::ParameterValue(3.0));
  node_->get_parameter(name_ + ".prune_forward", limits_->prune_forward);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "prune_forward: %.2f", limits_->prune_forward);

  node_->declare_parameter(name_ + ".prune_backward", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".prune_backward", limits_->prune_backward);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "prune_backward: %.2f", limits_->prune_backward);

  //@deceleration allow the robot to consider the deceleration based on the current_speed/deceleration_ratio
  //@for example, if current speed of robot is 1.2 then the considered min_vel will be 0.6 instead of current_speed-dt*acc
  node_->declare_parameter(name_ + ".deceleration_ratio", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".deceleration_ratio", limits_->deceleration_ratio);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "deceleration_ratio: %.2f", limits_->deceleration_ratio);

  if(limits_->min_vel_x<0)
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "The min velocity of the robot should be positive!");

  /*Motor constraint*/
  node_->declare_parameter(name_ + ".use_motor_constraint", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".use_motor_constraint", limits_->use_motor_constraint);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "use_motor_constraint: %d", limits_->use_motor_constraint);

  node_->declare_parameter(name_ + ".max_motor_shaft_rpm", rclcpp::ParameterValue(3000.0));
  node_->get_parameter(name_ + ".max_motor_shaft_rpm", limits_->max_motor_shaft_rpm);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_motor_shaft_rpm: %.2f", limits_->max_motor_shaft_rpm);

  node_->declare_parameter(name_ + ".wheel_diameter", rclcpp::ParameterValue(0.15));
  node_->get_parameter(name_ + ".wheel_diameter", limits_->wheel_diameter);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "wheel_diameter: %.2f", limits_->wheel_diameter);

  node_->declare_parameter(name_ + ".gear_ratio", rclcpp::ParameterValue(30.0));
  node_->get_parameter(name_ + ".gear_ratio", limits_->gear_ratio);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "gear_ratio: %.2f", limits_->gear_ratio);

  node_->declare_parameter(name_ + ".robot_radius", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".robot_radius", limits_->robot_radius);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "robot_radius: %.2f", limits_->robot_radius);

  //@initial params
  params_ = std::make_shared<trajectory_generators::OmniTrajectoryGeneratorParams>();

  node_->declare_parameter(name_ + ".controller_frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".controller_frequency", params_->controller_frequency);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "controller_frequency: %.2f", params_->controller_frequency);

  node_->declare_parameter(name_ + ".sim_time", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".sim_time", params_->sim_time);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_time: %.2f", params_->sim_time);

  node_->declare_parameter(name_ + ".linear_x_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".linear_x_sample", params_->linear_x_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "linear_x_sample: %.2f", params_->linear_x_sample);

  node_->declare_parameter(name_ + ".linear_y_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".linear_y_sample", params_->linear_y_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "linear_y_sample: %.2f", params_->linear_y_sample);

  node_->declare_parameter(name_ + ".angular_z_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".angular_z_sample", params_->angular_z_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "angular_z_sample: %.2f", params_->angular_z_sample);

  node_->declare_parameter(name_ + ".sim_granularity", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".sim_granularity", params_->sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_granularity: %.2f", params_->sim_granularity);

  node_->declare_parameter(name_ + ".angular_sim_granularity", rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".angular_sim_granularity", params_->angular_sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "angular_sim_granularity: %.2f", params_->angular_sim_granularity);

  //@ parse cuboid
  /*
  for(int i=1;i<9;i++){
    std::string s = ".cuboid.p" + std::to_string(i);

    node_->declare_parameter(name_ + s, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param = node_->get_parameter(name_ + s);
    auto p = cuboid_param.as_double_array();
    pcl::PointXYZ pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    params_->cuboid.push_back(pt);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid %.2f, %.2f, %.2f", pt.x, pt.y, pt.z);
  }
  */
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<double> p;

  node_->declare_parameter(name_ + ".cuboid.flb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_flb= node_->get_parameter(name_ + ".cuboid.flb");
  p = cuboid_flb.as_double_array();
  pcl::PointXYZ pt_flb;
  pt_flb.x = p[0];pt_flb.y = p[1];pt_flb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid flb: %.2f, %.2f, %.2f", pt_flb.x, pt_flb.y, pt_flb.z);

  node_->declare_parameter(name_ + ".cuboid.frb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_frb= node_->get_parameter(name_ + ".cuboid.frb");
  p = cuboid_frb.as_double_array();
  pcl::PointXYZ pt_frb;
  pt_frb.x = p[0];pt_frb.y = p[1];pt_frb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid frb: %.2f, %.2f, %.2f", pt_frb.x, pt_frb.y, pt_frb.z);

  node_->declare_parameter(name_ + ".cuboid.flt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_flt= node_->get_parameter(name_ + ".cuboid.flt");
  p = cuboid_flt.as_double_array();
  pcl::PointXYZ pt_flt;
  pt_flt.x = p[0];pt_flt.y = p[1];pt_flt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid flt: %.2f, %.2f, %.2f", pt_flt.x, pt_flt.y, pt_flt.z);

  node_->declare_parameter(name_ + ".cuboid.frt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_frt= node_->get_parameter(name_ + ".cuboid.frt");
  p = cuboid_frt.as_double_array();
  pcl::PointXYZ pt_frt;
  pt_frt.x = p[0];pt_frt.y = p[1];pt_frt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid frt: %.2f, %.2f, %.2f", pt_frt.x, pt_frt.y, pt_frt.z);

  node_->declare_parameter(name_ + ".cuboid.blb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_blb= node_->get_parameter(name_ + ".cuboid.blb");
  p = cuboid_blb.as_double_array();
  pcl::PointXYZ pt_blb;
  pt_blb.x = p[0];pt_blb.y = p[1];pt_blb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid blb: %.2f, %.2f, %.2f", pt_blb.x, pt_blb.y, pt_blb.z);

  node_->declare_parameter(name_ + ".cuboid.brb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_brb= node_->get_parameter(name_ + ".cuboid.brb");
  p = cuboid_brb.as_double_array();
  pcl::PointXYZ pt_brb;
  pt_brb.x = p[0];pt_brb.y = p[1];pt_brb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid brb: %.2f, %.2f, %.2f", pt_brb.x, pt_brb.y, pt_brb.z);

  node_->declare_parameter(name_ + ".cuboid.blt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_blt= node_->get_parameter(name_ + ".cuboid.blt");
  p = cuboid_blt.as_double_array();
  pcl::PointXYZ pt_blt;
  pt_blt.x = p[0];pt_blt.y = p[1];pt_blt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid blt: %.2f, %.2f, %.2f", pt_blt.x, pt_blt.y, pt_blt.z);

  node_->declare_parameter(name_ + ".cuboid.brt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_brt= node_->get_parameter(name_ + ".cuboid.brt");
  p = cuboid_brt.as_double_array();
  pcl::PointXYZ pt_brt;
  pt_brt.x = p[0];pt_brt.y = p[1];pt_brt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid brt: %.2f, %.2f, %.2f", pt_brt.x, pt_brt.y, pt_brt.z);

  params_->cuboid.push_back(pt_blb);
  params_->cuboid.push_back(pt_brb);
  params_->cuboid.push_back(pt_blt);
  params_->cuboid.push_back(pt_flb);
  params_->cuboid.push_back(pt_brt);
  params_->cuboid.push_back(pt_frt);
  params_->cuboid.push_back(pt_flt);
  params_->cuboid.push_back(pt_frb);
  //@ push the point by following sequence, because when doing the point in cuboid test we leverage blb/brb/blt/flb
  //@ back left top = blt; back right bottom = brb;
  //
  //          -------
  //         /|    /|
  //        / |   / |
  //     blt------- |
  //        | /flb| /
  //        |/    |/
  //     blb-------brb

  if(params_->cuboid.size()!=8){
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "Cuboid is essential.");
  }

}

void OmniSimpleTrajectoryGeneratorTheory::initialise(){
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  double max_vel_th = limits_->max_vel_theta;
  double min_vel_th = -1.0 * max_vel_th;
  Eigen::Vector3f acc_lim = limits_->getAccLimits();
  next_sample_index_ = 0;
  sample_params_.clear();
  admitted_axis_ = -1;
  admitted_sign_ = 1;
  admitted_speed_cap_ = 0.0;
  desired_x_speed_cap_ = 0.0;
  shared_data_->single_axis_avoidance_active_ = false;
  shared_data_->single_axis_avoidance_has_safe_command_ = false;

  double min_vel_x = limits_->min_vel_x;
  double max_vel_x = limits_->max_vel_x;

  double min_vel_y = limits_->min_vel_y;
  double max_vel_y = limits_->max_vel_y;

  // if sampling number is zero in any dimension, we don't generate samples generically
  if (params_->linear_x_sample * params_->angular_z_sample > 0) {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

    // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
    double sim_period = 1.0/params_->controller_frequency;
    

    max_vel[0] = std::min(max_vel_x, shared_data_->robot_state_.twist.twist.linear.x + acc_lim[0] * sim_period);
    max_vel[1] = std::min(max_vel_y, shared_data_->robot_state_.twist.twist.linear.y + acc_lim[1] * sim_period);
    max_vel[2] = std::min(max_vel_th, shared_data_->robot_state_.twist.twist.angular.z + acc_lim[2] * sim_period);

    min_vel[0] = std::max(min_vel_x, shared_data_->robot_state_.twist.twist.linear.x - acc_lim[0] * sim_period);
    min_vel[1] = std::max(min_vel_y, shared_data_->robot_state_.twist.twist.linear.y - acc_lim[1] * sim_period);
    min_vel[2] = std::max(min_vel_th, shared_data_->robot_state_.twist.twist.angular.z - acc_lim[2] * sim_period);
    
    
    if(shared_data_->robot_state_.twist.twist.linear.x >= max_vel_x/limits_->deceleration_ratio){
      //@ robot reach max speed at forward/backward
      min_vel[0] = std::max(min_vel_x, shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio);
    }
    else if(shared_data_->robot_state_.twist.twist.linear.x <= min_vel_x/limits_->deceleration_ratio){
      max_vel[0] = std::min(max_vel_x, shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio);
    }

    if(shared_data_->robot_state_.twist.twist.linear.y >= max_vel_y/limits_->deceleration_ratio){
      //@ robot reach max speed at forward/backward
      min_vel[1] = std::max(min_vel_y, shared_data_->robot_state_.twist.twist.linear.y/limits_->deceleration_ratio);
    }
    else if(shared_data_->robot_state_.twist.twist.linear.y <= min_vel_y/limits_->deceleration_ratio){
      max_vel[1] = std::min(max_vel_y, shared_data_->robot_state_.twist.twist.linear.y/limits_->deceleration_ratio);
    }

    // An overspeed measurement can make the reachable interval disjoint
    // from command limits. Request the nearest legal boundary and simulate
    // the actual braking transient below; never sample a reversed interval.
    const Eigen::Vector3f command_min(min_vel_x, min_vel_y, min_vel_th);
    const Eigen::Vector3f command_max(max_vel_x, max_vel_y, max_vel_th);
    for (int axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(min_vel[axis]) || !std::isfinite(max_vel[axis]) ||
          command_min[axis] > command_max[axis]) return;
      min_vel[axis] = std::clamp(min_vel[axis], command_min[axis], command_max[axis]);
      max_vel[axis] = std::clamp(max_vel[axis], command_min[axis], command_max[axis]);
      if (min_vel[axis] > max_vel[axis]) return;
    }
    // Samples are commanded setpoints, not instantaneous achieved velocities.
    // Permit braking toward zero even when measured overspeed excludes zero
    // from the one-cycle window. generateTrajectory still integrates the
    // measured velocity and finite deceleration; it never assumes instant stop.
    if (sample_braking_commands_ || single_axis_tracking_) {
      for (int axis = 0; axis < 3; ++axis) {
        if (command_min[axis] <= 0.0f && command_max[axis] >= 0.0f) {
          min_vel[axis] = std::min(min_vel[axis], 0.0f);
          max_vel[axis] = std::max(max_vel[axis], 0.0f);
        }
      }
    }
    // The single-axis policy decides only which scored command may be sent.
    // Candidate generation below always retains the complete vx * vy * wz
    // lattice so collision diagnostics and visualization are not collapsed to
    // the currently admitted axis.
    if (single_axis_tracking_) {
      std::vector<std::array<double,2>> path;
      for (const auto& p : shared_data_->prune_plan_.poses)
        path.push_back({p.pose.position.x, p.pose.position.y});
      const auto& pose = shared_data_->robot_pose_.transform;
      tf2::Quaternion q(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      auto errors = trackingErrors(path, pose.translation.x, pose.translation.y, yaw,
                                   shared_data_->path_heading_lookahead_);
      shared_data_->rotation_error_ = errors.heading;
      const auto& odom = shared_data_->robot_state_;
      const auto& v = odom.twist.twist;
      const auto stamp = rclcpp::Time(odom.header.stamp);
      const auto now = node_->now();
      const auto tf_stamp = rclcpp::Time(shared_data_->robot_pose_.header.stamp);
      const double age = (now - stamp).seconds();
      const double tf_age = (now - tf_stamp).seconds();
      const int64_t now_ns = now.nanoseconds();
      if ((avoidance_active_ || avoidance_returning_to_x_) &&
          (avoidance_started_ns_ <= 0 || now_ns < avoidance_started_ns_ ||
           static_cast<double>(now_ns - avoidance_started_ns_) * 1e-9 > avoidance_timeout_)) {
        RCLCPP_WARN(node_->get_logger().get_child(name_),
          "单轴避障超时 %.2f s，停车并恢复全局重规划", avoidance_timeout_);
        avoidance_active_ = false;
        avoidance_returning_to_x_ = false;
        avoidance_axis_ = -1;
        avoidance_x_clear_count_ = 0;
        avoidance_axis_blocked_count_ = 0;
        avoidance_started_ns_ = 0;
        avoidance_cooldown_until_ns_ = now_ns +
          static_cast<int64_t>(avoidance_retry_delay_ * 1e9);
      }
      desired_x_sign_ = errors.forward >= 0 ? 1 : -1;
      desired_x_speed_cap_ = std::max(std::abs(command_min[0]), std::abs(command_max[0]));
      if (limits_->max_vel_trans >= 0)
        desired_x_speed_cap_ = std::min(desired_x_speed_cap_, limits_->max_vel_trans);
      if (shared_data_->current_allowed_max_linear_speed_ > 0)
        desired_x_speed_cap_ = std::min(
          desired_x_speed_cap_, shared_data_->current_allowed_max_linear_speed_);
      const int forced_axis = avoidance_active_ ? avoidance_axis_ :
        avoidance_returning_to_x_ ? 0 : -1;
      const int forced_sign = avoidance_active_ ? avoidance_sign_ : desired_x_sign_;
      admitted_axis_ = axis_policy_.choose(errors, {v.linear.x, v.linear.y, v.angular.z},
        stamp.nanoseconds(), dddmr_sys_core::motionTimestampFresh(stamp.nanoseconds(), age) &&
        dddmr_sys_core::motionTimestampFresh(tf_stamp.nanoseconds(), tf_age),
        axis_yaw_enter_, axis_yaw_exit_, axis_lateral_enter_, axis_lateral_exit_,
        true, forced_axis, forced_sign);  // Forced avoidance still uses stop-before-switch.
      admitted_sign_ = admitted_axis_ >= 0 ? axis_policy_.sign() : 1;
      if (admitted_axis_ < 0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "停车原因：%s; heading_error=%.4f rad, lateral_error=%.4f m, forward_error=%.4f m, measured=(%.4f,%.4f,%.4f), odom_age=%.3f s, tf_age=%.3f s, robot_yaw=%.4f rad, reference=(%.3f,%.3f)->(%.3f,%.3f), path_points=%zu",
          axis_policy_.stopReason(), errors.heading, errors.lateral, errors.forward,
          v.linear.x, v.linear.y, v.angular.z, age, tf_age, yaw,
          errors.reference_start[0], errors.reference_start[1],
          errors.reference_end[0], errors.reference_end[1], path.size());
      } else {
        admitted_speed_cap_ = admitted_axis_ == 2 ? max_vel_th :
          std::max(std::abs(command_min[admitted_axis_]),
                   std::abs(command_max[admitted_axis_]));
        if (admitted_axis_ != 2 && limits_->max_vel_trans >= 0)
          admitted_speed_cap_ = std::min(admitted_speed_cap_, limits_->max_vel_trans);
        if (admitted_axis_ != 2 && shared_data_->current_allowed_max_linear_speed_ > 0)
          admitted_speed_cap_ = std::min(
            admitted_speed_cap_, shared_data_->current_allowed_max_linear_speed_);
        if (admitted_axis_ == 2 && !avoidance_active_) {
          const double cap = limits_->min_vel_theta >= max_vel_th ? max_vel_th :
            std::min(max_vel_th, std::max(limits_->min_vel_theta, std::abs(errors.heading)));
          admitted_speed_cap_ = cap;
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "旋转减速：剩余角度=%.2f deg, 候选角速度上限=%.3f rad/s",
            errors.heading * 180.0 / std::acos(-1.0), cap);
        }
      }
    }
    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    trajectory_generators::VelocityIterator x_it(min_vel[0], max_vel[0], params_->linear_x_sample);
    trajectory_generators::VelocityIterator y_it(min_vel[1], max_vel[1], params_->linear_y_sample);
    trajectory_generators::VelocityIterator th_it(min_vel[2], max_vel[2], params_->angular_z_sample);
    for(; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) {
        vel_samp[1] = y_it.getVelocity();

        for(; !th_it.isFinished(); th_it++) {
          vel_samp[2] = th_it.getVelocity();
          //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
          if(isMotorConstraintSatisfied(vel_samp))
            sample_params_.push_back(vel_samp);
        }
        th_it.reset();
      }
      y_it.reset();
    }
    //RCLCPP_INFO(node_->get_logger().get_child(name_), "%.2f, %.2f, %.2f", vel.twist.twist.linear.x, vel.twist.twist.linear.y, vel.twist.twist.angular.z);
  }    
}

bool OmniSimpleTrajectoryGeneratorTheory::isMotorConstraintSatisfied(Eigen::Vector3f& vel_samp){
  
  //@ if we dont want motor constraint, return constraint is atisfied
  if(!limits_->use_motor_constraint)
    return true;

  return true;

  //@ TODO: Omni kinematics required
}

size_t OmniSimpleTrajectoryGeneratorTheory::getSamplingSize(){
  return sample_params_.size();
}

void OmniSimpleTrajectoryGeneratorTheory::getSamplingTrajectoryByIndex(size_t index, base_trajectory::Trajectory& _traj){
  generateTrajectory(sample_params_[index], _traj);
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool OmniSimpleTrajectoryGeneratorTheory::generateTrajectory(
      Eigen::Vector3f sample_target_vel,
      base_trajectory::Trajectory& traj) {

  //@ assign actuator type to trajectory, so that when move base publishing the cmd_vel,
  //@ the correct publisher will be used i.e., geometry/twist or ackermann
  traj.actuator_type_ = actuator_type_;
  
  Eigen::Affine3d pos_af3 = tf2::transformToEigen(shared_data_->robot_pose_);
  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
  double eps = 1e-4;
  traj.cost_ = 0.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoses();
  const auto& measured = shared_data_->robot_state_.twist.twist;
  Eigen::Vector3f initial_velocity(measured.linear.x, measured.linear.y, measured.angular.z);
  if (!sample_target_vel.allFinite() || !initial_velocity.allFinite() ||
      sample_target_vel[0] < limits_->min_vel_x - eps ||
      sample_target_vel[0] > limits_->max_vel_x + eps ||
      sample_target_vel[1] < limits_->min_vel_y - eps ||
      sample_target_vel[1] > limits_->max_vel_y + eps ||
      std::abs(sample_target_vel[2]) > limits_->max_vel_theta + eps) return false;
  const Eigen::Vector3f acceleration = limits_->getAccLimits();
  if (!acceleration.allFinite() || (acceleration.array() <= 0).any() ||
      !std::isfinite(limits_->deceleration_ratio) || limits_->deceleration_ratio <= 0 ||
      !std::isfinite(params_->sim_time) || params_->sim_time <= 0 ||
      !std::isfinite(params_->sim_granularity) || params_->sim_granularity <= 0 ||
      !std::isfinite(params_->angular_sim_granularity) || params_->angular_sim_granularity <= 0 ||
      !std::isfinite(params_->controller_frequency) || params_->controller_frequency <= 0) return false;

  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Trajectory by state x: %.2f, y: %.2f, w: %.2f", sample_target_vel[0], sample_target_vel[1], sample_target_vel[2]);

  // make sure that the robot would at least be moving with one of
  // the required minimum velocities for translation and rotation (if set)
  const bool braking_stop = (sample_braking_commands_ || single_axis_tracking_) && sample_target_vel.isZero(0.0f);
  if (!braking_stop && (limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) {
    return false;
  }
  // make sure we do not exceed max diagonal (x+y) translational velocity (if set)
  if (limits_->max_vel_trans >=0 && vmag - eps > limits_->max_vel_trans) {
    return false;
  }

  // allow max speed to be changed by speed zone/perception features
  if(shared_data_->current_allowed_max_linear_speed_>0.0){
    if (vmag - eps > shared_data_->current_allowed_max_linear_speed_) {
      return false;
    }
  }

  int num_steps;

  //compute the number of steps we must take along this trajectory to be "safe"
  // Bound both current and target speeds, including mixed-axis transients.
  double sim_time_distance = std::hypot(
      std::max(std::abs(initial_velocity[0]), std::abs(sample_target_vel[0])),
      std::max(std::abs(initial_velocity[1]), std::abs(sample_target_vel[1]))) * params_->sim_time;
  double sim_time_angle = std::max(std::abs(initial_velocity[2]), std::abs(sample_target_vel[2])) * params_->sim_time;
  num_steps =
      ceil(std::max(sim_time_distance / params_->sim_granularity,
          sim_time_angle / params_->angular_sim_granularity));
  

  if (num_steps == 0 && !braking_stop) {
    return false;
  }
  // Low-speed startup still needs a trajectory, not a single endpoint.
  // Increasing subdivision preserves the horizon and tightens collision checks.
  num_steps = std::max(2, num_steps);
  num_steps = std::max(num_steps, static_cast<int>(std::ceil(params_->sim_time * params_->controller_frequency)));

  const double dt = params_->sim_time / num_steps;
  const int command_steps = num_steps;
  int reaction_steps = 0;
  if (sample_braking_commands_) {
    reaction_steps = static_cast<int>(std::ceil(braking_reaction_time_ / dt));
    double stopping_time = 0;
    for (int axis=0; axis<3; ++axis)
      stopping_time = std::max(stopping_time,
        static_cast<double>(std::max(std::abs(initial_velocity[axis]), std::abs(sample_target_vel[axis])) /
        (acceleration[axis] * limits_->deceleration_ratio)));
    num_steps += reaction_steps + static_cast<int>(std::ceil(stopping_time / dt));
  }
  traj.time_delta_ = dt;

  //compute a timestep
  //double dt = 0.1;
  //int num_steps = 30;
  //traj.time_delta_ = dt;

  
  Eigen::Vector3f loop_vel;

  // assuming sample_vel is our target velocity within acc limits for one timestep
  loop_vel = initial_velocity;
  traj.xv_     = sample_target_vel[0];
  traj.yv_     = sample_target_vel[1];
  traj.thetav_ = sample_target_vel[2];
  
  /*We first create trajectory based on robot_frame, then we use affine to transform it to global frame*/
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  //simulate the trajectory and check for collisions, updating costs along the way
  // Include the current pose, reaction delay, command horizon and full stop tail.
  for (int i = 0; i <= num_steps; ++i) {
    if (i > 0) {
      Eigen::Vector3f average_velocity;
      const int step = i-1;
      for (int axis=0; axis<3; ++axis) {
        double velocity=loop_vel[axis];
        if (step < reaction_steps) {
          average_velocity[axis]=velocity;
        } else {
          const double target = step < reaction_steps+command_steps ? sample_target_vel[axis] : 0.0;
          average_velocity[axis]=integrateVelocity(velocity, target, acceleration[axis],
                                                  limits_->deceleration_ratio, dt)/dt;
          loop_vel[axis]=velocity;
        }
      }
      pos = computeNewPositions(pos, average_velocity, dt);
    }

    /*transform back to global frame*/
    Eigen::Affine3d trans_gbl2traj_af3;

    
    Eigen::Affine3d trans_b2traj_af3(Eigen::AngleAxisd(pos[2], Eigen::Vector3d::UnitZ()));
    trans_b2traj_af3.translation().x() = pos[0];
    trans_b2traj_af3.translation().y() = pos[1];
    
    /*
    tf2::Quaternion tf2_rotation;
    tf2_rotation.setRPY( 0, 0, pos[2]); 
    tf2_rotation.normalize();
    geometry_msgs::TransformStamped trans_b2traj;
    trans_b2traj.transform.translation.x = pos[0];
    trans_b2traj.transform.translation.y = pos[1];
    trans_b2traj.transform.rotation.x = tf2_rotation.x();
    trans_b2traj.transform.rotation.x = tf2_rotation.y();
    trans_b2traj.transform.rotation.x = tf2_rotation.z();
    trans_b2traj.transform.rotation.x = tf2_rotation.w();
    Eigen::Affine3d trans_b2traj_af3 = tf2::transformToEigen(trans_b2traj);
    */
    trans_gbl2traj_af3 = pos_af3*trans_b2traj_af3;
    geometry_msgs::msg::TransformStamped trans_gbl2traj_ = tf2::eigenToTransform (trans_gbl2traj_af3);
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header = shared_data_->robot_pose_.header;
    ros_pose.pose.position.x = trans_gbl2traj_.transform.translation.x;
    ros_pose.pose.position.y = trans_gbl2traj_.transform.translation.y;
    ros_pose.pose.position.z = trans_gbl2traj_.transform.translation.z;
    ros_pose.pose.orientation = trans_gbl2traj_.transform.rotation;

    pcl::PointCloud<pcl::PointXYZ> pc_out;
    pcl::transformPointCloud(params_->cuboid, pc_out, trans_gbl2traj_af3);
    
    base_trajectory::cuboid_min_max_t cuboid_min_max;
    pcl::getMinMax3D(pc_out, cuboid_min_max.first, cuboid_min_max.second);

    if(!traj.addPoseCuboid(ros_pose, pc_out, cuboid_min_max)){
      return false;
    }

  } // end for simulation steps

  return true; // trajectory has at least one point
}

Eigen::Vector3f OmniSimpleTrajectoryGeneratorTheory::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

void OmniSimpleTrajectoryGeneratorTheory::expertScoring(std::vector<base_trajectory::Trajectory>& accepted_trajectories,
                                            std::map<std::string, std::vector<base_trajectory::Trajectory>>& rejected_trajectories, 
                                              base_trajectory::Trajectory& best_traj){
  if (!single_axis_tracking_) {
    TrajectoryGeneratorTheory::expertScoring(accepted_trajectories, rejected_trajectories, best_traj);
    return;
  }
  std::vector<std::array<double,4>> candidates;
  candidates.reserve(accepted_trajectories.size());
  for (const auto& t : accepted_trajectories)
    candidates.push_back({t.xv_,t.yv_,t.thetav_,t.cost_});
  best_traj.cost_ = -1;
  shared_data_->single_axis_avoidance_active_ =
    avoidance_active_ || avoidance_returning_to_x_;
  shared_data_->single_axis_avoidance_has_safe_command_ = false;

  const double y_cap = limits_->max_vel_trans >= 0 ?
    std::min(std::max(std::abs(limits_->min_vel_y), std::abs(limits_->max_vel_y)),
             limits_->max_vel_trans) :
    std::max(std::abs(limits_->min_vel_y), std::abs(limits_->max_vel_y));
  const double yaw_cap = limits_->max_vel_theta;
  const int x_motion = bestSingleAxisMotion(
    candidates, 0, desired_x_sign_, desired_x_speed_cap_);
  const auto alternative = choosePureAvoidanceMotion(candidates, y_cap, yaw_cap);
  int best = chooseSingleAxisMotionOrBrake(
    candidates, admitted_axis_, admitted_sign_, admitted_speed_cap_);

  if (!axis_obstacle_avoidance_) {
    if (best >= 0) best_traj = accepted_trajectories[best];
  } else if (!avoidance_active_ && !avoidance_returning_to_x_) {
    const bool x_was_selected = admitted_axis_ == 0;
    if (x_was_selected && x_motion < 0 && alternative.index >= 0 &&
        node_->now().nanoseconds() >= avoidance_cooldown_until_ns_) {
      avoidance_active_ = true;
      avoidance_axis_ = alternative.axis;
      avoidance_sign_ = alternative.sign;
      avoidance_x_clear_count_ = 0;
      avoidance_axis_blocked_count_ = 0;
      avoidance_started_ns_ = node_->now().nanoseconds();
      shared_data_->single_axis_avoidance_active_ = true;
      shared_data_->single_axis_avoidance_has_safe_command_ = true;
      // The alternative is known safe, but this cycle must command the brake;
      // the policy admits it only after three new stopped odometry samples.
      best = chooseSingleAxisMotionOrBrake(candidates, -1, 1, 0.0);
      RCLCPP_WARN(node_->get_logger().get_child(name_),
        "纯 X 无安全候选，进入单轴避障：先停车，再锁定 %s%s",
        avoidance_axis_ == 1 ? "Y" : "Yaw", avoidance_sign_ > 0 ? "+" : "-");
    }
    if (best >= 0) best_traj = accepted_trajectories[best];
  } else if (avoidance_returning_to_x_) {
    if (x_motion < 0) {
      ++avoidance_axis_blocked_count_;
      if (avoidance_axis_blocked_count_ >= avoidance_switch_cycles_) {
        avoidance_returning_to_x_ = false;
        avoidance_axis_blocked_count_ = 0;
        if (alternative.index >= 0) {
          avoidance_active_ = true;
          avoidance_axis_ = alternative.axis;
          avoidance_sign_ = alternative.sign;
          avoidance_x_clear_count_ = 0;
          shared_data_->single_axis_avoidance_active_ = true;
          shared_data_->single_axis_avoidance_has_safe_command_ = true;
          RCLCPP_WARN(node_->get_logger().get_child(name_),
            "切回 X 时安全候选消失，重新锁定 %s%s",
            avoidance_axis_ == 1 ? "Y" : "Yaw", avoidance_sign_ > 0 ? "+" : "-");
        } else {
          shared_data_->single_axis_avoidance_active_ = false;
        }
      } else {
        shared_data_->single_axis_avoidance_active_ = true;
        // Stay stopped for a bounded number of cycles while collision scoring
        // settles. A known-safe alternative keeps this transition recoverable.
        shared_data_->single_axis_avoidance_has_safe_command_ = alternative.index >= 0;
      }
      best = chooseSingleAxisMotionOrBrake(candidates, -1, 1, 0.0);
    } else if (admitted_axis_ == 0 && admitted_sign_ == desired_x_sign_) {
      avoidance_axis_blocked_count_ = 0;
      best = x_motion;
      avoidance_returning_to_x_ = false;
      avoidance_axis_ = -1;
      avoidance_started_ns_ = 0;
      RCLCPP_INFO(node_->get_logger().get_child(name_),
        "单轴避障完成，已停稳并恢复纯 X 指令");
    } else {
      best = chooseSingleAxisMotionOrBrake(candidates, -1, 1, 0.0);
      shared_data_->single_axis_avoidance_active_ = true;
      shared_data_->single_axis_avoidance_has_safe_command_ = true;
    }
    if (best >= 0) best_traj = accepted_trajectories[best];
  } else {
    if (x_motion >= 0) ++avoidance_x_clear_count_;
    else avoidance_x_clear_count_ = 0;
    if (avoidance_x_clear_count_ >= avoidance_clear_cycles_) {
      avoidance_active_ = false;
      avoidance_returning_to_x_ = true;
      avoidance_x_clear_count_ = 0;
      avoidance_axis_blocked_count_ = 0;
      best = chooseSingleAxisMotionOrBrake(candidates, -1, 1, 0.0);
      shared_data_->single_axis_avoidance_active_ = true;
      shared_data_->single_axis_avoidance_has_safe_command_ = true;
      RCLCPP_INFO(node_->get_logger().get_child(name_),
        "纯 X 连续 %d 轮恢复安全，先停车再切回 X", avoidance_clear_cycles_);
    } else {
      const double locked_cap = avoidance_axis_ == 1 ? y_cap : yaw_cap;
      const int locked_motion = bestSingleAxisMotion(
        candidates, avoidance_axis_, avoidance_sign_, locked_cap);
      if (locked_motion >= 0) {
        avoidance_axis_blocked_count_ = 0;
        shared_data_->single_axis_avoidance_has_safe_command_ = true;
        // During stop confirmation admitted_axis_ is -1, so keep braking even
        // though the future locked command has already passed collision scoring.
        best = admitted_axis_ == avoidance_axis_ && admitted_sign_ == avoidance_sign_ ?
          locked_motion : chooseSingleAxisMotionOrBrake(candidates, -1, 1, 0.0);
      } else {
        ++avoidance_axis_blocked_count_;
        best = chooseSingleAxisMotionOrBrake(candidates, -1, 1, 0.0);
        if (alternative.index >= 0 &&
            avoidance_axis_blocked_count_ >= avoidance_switch_cycles_) {
          avoidance_axis_ = alternative.axis;
          avoidance_sign_ = alternative.sign;
          avoidance_axis_blocked_count_ = 0;
          shared_data_->single_axis_avoidance_has_safe_command_ = true;
          RCLCPP_WARN(node_->get_logger().get_child(name_),
            "当前避障轴无安全候选，停车后改锁定 %s%s",
            avoidance_axis_ == 1 ? "Y" : "Yaw", avoidance_sign_ > 0 ? "+" : "-");
        } else if (alternative.index >= 0) {
          shared_data_->single_axis_avoidance_has_safe_command_ = true;
        }
      }
      if (best >= 0) best_traj = accepted_trajectories[best];
    }
  }

  // Downstream blockage handling inspects collision-rejected commands. Keep
  // only commands that this single-axis state could actually execute, so a
  // colliding mixed-axis visualization candidate cannot request a replan.
  for (auto& rejected_by_critic : rejected_trajectories) {
    auto& trajectories = rejected_by_critic.second;
    trajectories.erase(std::remove_if(trajectories.begin(), trajectories.end(),
      [this](const base_trajectory::Trajectory& trajectory) {
        const bool admitted = singleAxisCommandAllowed(
          trajectory.xv_, trajectory.yv_, trajectory.thetav_,
          admitted_axis_, admitted_sign_, admitted_speed_cap_);
        const bool desired_x = (avoidance_active_ || avoidance_returning_to_x_) &&
          singleAxisCommandAllowed(trajectory.xv_, trajectory.yv_, trajectory.thetav_,
                                   0, desired_x_sign_, desired_x_speed_cap_);
        return !admitted && !desired_x;
      }), trajectories.end());
  }
}
}//end of name space
