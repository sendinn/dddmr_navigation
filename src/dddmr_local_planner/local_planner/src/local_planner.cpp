#include <dddmr_sys_core/motion_timestamp.h>
#include <trajectory_generators/path_sweep.h>
#include <trajectory_generators/braking_rollout.h>
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
#include <local_planner/local_planner.h>
#include <trajectory_generators/single_axis_tracking.h>

namespace local_planner {

Local_Planner::Local_Planner(const std::string& name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
  got_odom_ = false;
  last_valid_prune_plan_ = clock_->now();
}

void Local_Planner::initial(
      const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
      const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
      const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators){

  declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
  this->get_parameter("odom_topic", odom_topic_);
  RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic_.c_str());

  declare_parameter("odom_topic_qos", rclcpp::ParameterValue("best_effort"));
  this->get_parameter("odom_topic_qos", odom_topic_qos_);
  RCLCPP_INFO(this->get_logger(), "odom_topic_qos: %s", odom_topic_qos_.c_str());
  
  //@ subscibe steering angle/velocity from ackermann vehicle
  declare_parameter("steering_state_topic", rclcpp::ParameterValue("steering_state_topic"));
  this->get_parameter("steering_state_topic", steering_state_topic_);
  RCLCPP_INFO(this->get_logger(), "steering_state_topic: %s", steering_state_topic_.c_str());

  declare_parameter("forward_prune", rclcpp::ParameterValue(1.0));
  this->get_parameter("forward_prune", forward_prune_);
  RCLCPP_INFO(this->get_logger(), "forward_prune: %.2f", forward_prune_);

  declare_parameter("backward_prune", rclcpp::ParameterValue(0.5));
  this->get_parameter("backward_prune", backward_prune_);
  RCLCPP_INFO(this->get_logger(), "backward_prune: %.2f", backward_prune_);

  declare_parameter("heading_tracking_distance", rclcpp::ParameterValue(0.5));
  this->get_parameter("heading_tracking_distance", heading_tracking_distance_);
  RCLCPP_INFO(this->get_logger(), "heading_tracking_distance: %.2f", heading_tracking_distance_);

  declare_parameter("heading_align_angle", rclcpp::ParameterValue(0.5));
  this->get_parameter("heading_align_angle", heading_align_angle_);
  RCLCPP_INFO(this->get_logger(), "heading_align_angle: %.2f", heading_align_angle_);

  declare_parameter("prune_plane_timeout", rclcpp::ParameterValue(3.0));
  this->get_parameter("prune_plane_timeout", prune_plane_timeout_);
  RCLCPP_INFO(this->get_logger(), "prune_plane_timeout: %.2f", prune_plane_timeout_);

  declare_parameter("xy_goal_tolerance", rclcpp::ParameterValue(0.3));
  this->get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
  RCLCPP_INFO(this->get_logger(), "xy_goal_tolerance: %.2f", xy_goal_tolerance_);

  declare_parameter("yaw_goal_tolerance", rclcpp::ParameterValue(0.3));
  this->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
  RCLCPP_INFO(this->get_logger(), "yaw_goal_tolerance: %.2f", yaw_goal_tolerance_);

  declare_parameter("controller_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("controller_frequency", controller_frequency_);
  RCLCPP_INFO(this->get_logger(), "controller_frequency: %.2f", controller_frequency_);


  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  perception_3d_ros_ = perception_3d;
  mpc_critics_ros_ = mpc_critics;
  trajectory_generators_ros_ = trajectory_generators;

  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  parseCuboid(); //after robot_frame is got
  obstacle_replan_lookahead_ = declare_parameter<double>("obstacle_replan_lookahead", 0.0);
  if (!std::isfinite(obstacle_replan_lookahead_) || obstacle_replan_lookahead_<0 || obstacle_replan_lookahead_>5)
    throw std::invalid_argument("obstacle_replan_lookahead must be in [0,5] meters");
  
  pub_robot_cuboid_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_cuboid", 1);  
  pub_aggregate_observation_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aggregated_pc", 1);  
  pub_prune_plan_ = this->create_publisher<nav_msgs::msg::Path>("prune_plan", 1);
  pub_accepted_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("accepted_trajectory", 1);
  pub_local_trajectory_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("local_trajectory_candidates", 1);
  pub_best_trajectory_pose_ = this->create_publisher<geometry_msgs::msg::PoseArray>("best_trajectory", 2);
  pub_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("trajectory", 2);
  //pub_pc_normal_ = pnh_.advertise<visualization_msgs::MarkerArray>("normal_marker", 2, true);
  //pub_trajectory_cuboids_ = pnh_.advertise<sensor_msgs::PointCloud2>("trajectory_cuboids", 2, true);

  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  
  if(odom_topic_qos_=="reliable" || odom_topic_qos_=="Reliable"){
    odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(),
      std::bind(&Local_Planner::cbOdom, this, std::placeholders::_1), sub_options);
  }
  else{
    odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&Local_Planner::cbOdom, this, std::placeholders::_1), sub_options);
  }
  
  steering_state_ros_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      steering_state_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&Local_Planner::cbSteeringState, this, std::placeholders::_1), sub_options);

  //@Initial pcl ptr
  pcl_global_plan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree_global_plan_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
}

Local_Planner::~Local_Planner(){

  perception_3d_ros_.reset();
  mpc_critics_ros_.reset();
  trajectory_generators_ros_.reset();
  trajectories_.reset();
  tf2Buffer_.reset();

}

std::string Local_Planner::getControlFrame(){
  return perception_3d_ros_->getGlobalUtils()->getRobotFrame();
};

void Local_Planner::parseCuboid(){
  marker_edge_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getRobotFrame();;
  marker_edge_.header.stamp = clock_->now();
  marker_edge_.action = visualization_msgs::msg::Marker::ADD;
  marker_edge_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_edge_.pose.orientation.w = 1.0;
  marker_edge_.ns = "edges";
  marker_edge_.id = 3; marker_edge_.scale.x = 0.03;
  marker_edge_.color.r = 0.9; marker_edge_.color.g = 1; marker_edge_.color.b = 0; marker_edge_.color.a = 0.8;
  //@ parse cuboid, currently the cuboid in local planner is just for visualization
  RCLCPP_INFO(this->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<std::string> cuboid_vertex_queue = {"cuboid.flb", "cuboid.frb", "cuboid.flt", "cuboid.frt", "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.brt"};
  std::map<std::string, std::vector<double>> cuboid_vertex_parameter_map;

  for(auto it=cuboid_vertex_queue.begin(); it!=cuboid_vertex_queue.end();it++){
    std::vector<double> p;
    geometry_msgs::msg::Point pt;
    this->declare_parameter(*it, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param= this->get_parameter(*it);
    p = cuboid_param.as_double_array();
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
    cuboid_vertex_parameter_map[*it] = p;
  }
  RCLCPP_INFO(this->get_logger().get_child(name_), "Cuboid vertex are loaded, start to connect edges.");
  std::vector<std::string> cuboid_vertex_connect = {"cuboid.flb", "cuboid.blb", "cuboid.flt", "cuboid.blt", "cuboid.frb", "cuboid.brb", "cuboid.frt", "cuboid.brt",
                                                      "cuboid.flt", "cuboid.flb", "cuboid.frt", "cuboid.frb", "cuboid.blt", "cuboid.blb", "cuboid.brt", "cuboid.brb"};
  for(auto it=cuboid_vertex_connect.begin(); it!=cuboid_vertex_connect.end();it++){
    auto p = cuboid_vertex_parameter_map[*it];
    geometry_msgs::msg::Point pt;
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
  }
}

void Local_Planner::syncRobotState(nav_msgs::msg::Odometry& odom, ackermann_msgs::msg::AckermannDriveStamped& ackermann_drive_state){
  odom = robot_state_;
  ackermann_drive_state = ackermann_drive_state_;
}

void Local_Planner::cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_state_ = *msg;
  updateGlobalPose();
  got_odom_ = true;
}

void Local_Planner::cbSteeringState(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){
  ackermann_drive_state_ = *msg;
  updateGlobalPose();
}

double Local_Planner::getShortestAngleFromPose2RobotHeading(tf2::Transform m_pose){

  //@Transform trans_gbl2b_ to tf2; Get baselink to global, so that we later can get base_link2gbl * gbl2lastpose
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2b;
  tf2::fromMsg(trans_gbl2b_, tf2_trans_gbl2b);
  auto tf2_trans_gbl2b_inverse = tf2_trans_gbl2b.inverse();
  //@Get baselink to last pose
  tf2::Transform tf2_baselink2prunelastpose;
  tf2_baselink2prunelastpose.mult(tf2_trans_gbl2b_inverse, m_pose);
  //@Get RPY
  tf2::Matrix3x3 m(tf2_baselink2prunelastpose.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //@Although the test shows that yaw is already the shortest, we will use shortest_angular_distance anyway.
  yaw = angles::shortest_angular_distance(0.0, yaw);
  
  return yaw;

}

void Local_Planner::resetRotationReference() {
  path_heading_locked_ = false;
  ++trajectory_generators_ros_->getSharedDataPtr()->rotation_reference_epoch_;
}

bool Local_Planner::isInitialHeadingAligned(){

  // Identical XY reference and pruning as the single-axis tracking controller.
  prunePlan(std::max(forward_prune_, heading_tracking_distance_), backward_prune_);
  std::vector<std::array<double,2>> path;
  for (const auto& p : prune_plan_.poses)
    path.push_back({p.pose.position.x, p.pose.position.y});
  const auto& pose = trans_gbl2b_.transform;
  tf2::Quaternion q(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  const auto errors = trajectory_generators::trackingErrors(
    path, pose.translation.x, pose.translation.y, yaw, heading_tracking_distance_);
  if (!errors.valid) return false;
  mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = errors.heading;
  return std::abs(errors.heading) <= heading_align_angle_;
}

bool Local_Planner::isGoalHeadingAligned(){

  if(global_plan_.empty()){
    return false;
  }

  geometry_msgs::msg::PoseStamped final_pose;
  final_pose = global_plan_.back();

  geometry_msgs::msg::TransformStamped final_pose_ts;
  final_pose_ts.header = final_pose.header;
  final_pose_ts.transform.translation.x = final_pose.pose.position.x;
  final_pose_ts.transform.translation.y = final_pose.pose.position.y;
  final_pose_ts.transform.translation.z = final_pose.pose.position.z;
  final_pose_ts.transform.rotation.x = final_pose.pose.orientation.x;
  final_pose_ts.transform.rotation.y = final_pose.pose.orientation.y;
  final_pose_ts.transform.rotation.z = final_pose.pose.orientation.z;
  final_pose_ts.transform.rotation.w = final_pose.pose.orientation.w;
  tf2::Stamped<tf2::Transform> tf2_trans_gbl2goal;
  tf2::fromMsg(final_pose_ts, tf2_trans_gbl2goal);  

  //@Update the value to critics that allow the robot to turn by shortest angle
  double yaw = getShortestAngleFromPose2RobotHeading(tf2_trans_gbl2goal);
  mpc_critics_ros_->getSharedDataPtr()->heading_deviation_ = yaw;
  
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Heading difference to goal is %.2f", yaw);

  if(fabs(yaw) <= yaw_goal_tolerance_)
    return true;
  else
    return false;
}

bool Local_Planner::isGoalReached(){
  if(global_plan_.empty()){
    return false;
  }
  geometry_msgs::msg::PoseStamped final_pose;
  final_pose = global_plan_.back();
  double dx = trans_gbl2b_.transform.translation.x - final_pose.pose.position.x;
  double dy = trans_gbl2b_.transform.translation.y - final_pose.pose.position.y;
  // Goal is projected onto the ground; base_link height must not consume XY tolerance.
  double distance = std::hypot(dx, dy);
  if(distance <= xy_goal_tolerance_)
    return true;
  else
    return false;
}

void Local_Planner::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan) {

  if(orig_global_plan.size()<3){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Size of global plan is smaller than 3.");
    return;
  }

  global_plan_.clear();
  global_plan_ = orig_global_plan;

  pcl_global_plan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto gbl_it = global_plan_.begin(); gbl_it!=global_plan_.end();gbl_it++){
    pcl::PointXYZ pt;
    pt.x = (*gbl_it).pose.position.x;
    pt.y = (*gbl_it).pose.position.y;
    pt.z = (*gbl_it).pose.position.z;
    pcl_global_plan_->push_back(pt);
  }

  kdtree_global_plan_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kdtree_global_plan_->setInputCloud (pcl_global_plan_);
  RCLCPP_INFO_THROTTLE(this->get_logger().get_child(name_), *clock_, 10000, "Recieve new global plan.");
  //RCLCPP_INFO(this->get_logger().get_child(name_), "Recieve new global plan: %.2f, %.2f", 
  //    global_plan_.back().pose.position.x, global_plan_.back().pose.position.y);
}

double Local_Planner::getDistanceBTWPoseStamp(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b){

  double dx = a.pose.position.x-b.pose.position.x;
  double dy = a.pose.position.y-b.pose.position.y;
  double dz = a.pose.position.z-b.pose.position.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

void Local_Planner::updateGlobalPose(){
  try
  {
    trans_gbl2b_ = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "%s: %s", name_.c_str(),e.what());
  }
  robot_cuboid_.markers.clear();
  marker_edge_.header.stamp = trans_gbl2b_.header.stamp;
  robot_cuboid_.markers.push_back(marker_edge_);
  pub_robot_cuboid_->publish(robot_cuboid_);
}

geometry_msgs::msg::TransformStamped Local_Planner::getGlobalPose(){
  return trans_gbl2b_;
}

void Local_Planner::prunePlan(double forward_distance, double backward_distance){

  prune_plan_.poses.clear();
  pcl_prune_plan_.clear();
  if(pcl_global_plan_->points.size()<3)
    return;

  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  pcl::PointXYZ robot_pose;
  robot_pose.x = trans_gbl2b_.transform.translation.x;
  robot_pose.y = trans_gbl2b_.transform.translation.y;
  robot_pose.z = trans_gbl2b_.transform.translation.z;

  if ( kdtree_global_plan_->nearestKSearch (robot_pose, 1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0 ){
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "Ready to fix some exception here.");
    return;
  }


  if(sqrt(pointNKNSquaredDistance[0])>1.0){
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "Deviate from plan, fix some exception here.");
    //@ consider to clear prune_plan in model_shared_data?
    return;
  }

  //@ backward check
  geometry_msgs::msg::PoseStamped last_pose = global_plan_[pointIdxNKNSearch[0]];
  for(int i=pointIdxNKNSearch[0]; i>=0; i--){
    prune_plan_.poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x; pt.y = global_plan_[i].pose.position.y; pt.z = global_plan_[i].pose.position.z;
    pt.intensity = -1; //@ we tag backward plan as negative for path_blocked_strategy(plugin) to distinguish the backward pose
    pcl_prune_plan_.points.push_back(pt);
    if(i<pointIdxNKNSearch[0]){
      backward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    if(backward_distance<0)
      break;
  }
  
  std::reverse(prune_plan_.poses.begin(),prune_plan_.poses.end()); 

  //@ forward check
  for(int i=pointIdxNKNSearch[0];i<global_plan_.size();i++){
    prune_plan_.poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x; pt.y = global_plan_[i].pose.position.y; pt.z = global_plan_[i].pose.position.z;
    if(i == 0){
      pt.intensity = 0;
    }
    else{
      pt.intensity = 1;
    }
    pcl_prune_plan_.points.push_back(pt);

    if(i>pointIdxNKNSearch[0]){
      forward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    if(forward_distance<0)
      break;
  }
  
  prune_plan_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  prune_plan_.header.stamp = clock_->now();
  pub_prune_plan_->publish(prune_plan_);
  last_valid_prune_plan_ = clock_->now();
  //RCLCPP_DEBUG(this->get_logger().get_child(name_), "%lu",prune_plan_.poses.size());
}

bool Local_Planner::forwardPathBlocked() {
  if (obstacle_replan_lookahead_<=0 || marker_edge_.points.size()<8) return false;
  std::vector<std::array<double,3>> path;
  for (const auto& p : global_plan_)
    path.push_back({p.pose.position.x,p.pose.position.y,p.pose.position.z});
  const auto& t=trans_gbl2b_.transform;
  auto samples=trajectory_generators::forwardPathSamples(path,t.translation.x,t.translation.y,
                                                        t.translation.z,obstacle_replan_lookahead_,.05,heading_tracking_distance_);
  tf2::Quaternion current(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);
  double roll,pitch,yaw;
  tf2::Matrix3x3(current).getRPY(roll,pitch,yaw);
  std::array<double,3> lo{1e9,1e9,1e9},hi{-1e9,-1e9,-1e9};
  double radius=0;
  for(size_t i=0;i<8;++i) {
    const auto& p=marker_edge_.points[i];
    const std::array<double,3> v{p.x,p.y,p.z};
    for(int a=0;a<3;++a) {lo[a]=std::min(lo[a],v[a]);hi[a]=std::max(hi[a],v[a]);}
    radius=std::max(radius,std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z));
  }
  auto data=mpc_critics_ros_->getSharedDataPtr();
  for(const auto& sample:samples) {
    tf2::Quaternion q; q.setRPY(roll,pitch,sample[3]);
    tf2::Transform frame(q,tf2::Vector3(sample[0],sample[1],sample[2]));
    const auto inverse=frame.inverse();
    pcl::PointXYZI center;center.x=sample[0];center.y=sample[1];center.z=sample[2];
    std::vector<int> indices;std::vector<float> distances;
    data->pcl_perception_kdtree_->radiusSearch(center,radius,indices,distances);
    for(int index:indices) {
      const auto& p=data->pcl_perception_->points[index];
      const auto local=inverse*tf2::Vector3(p.x,p.y,p.z);
      if(trajectory_generators::pointInBox({local.x(),local.y(),local.z()},lo,hi)) {
        RCLCPP_WARN_THROTTLE(get_logger(),*clock_,1000,
          "提前避障：前方路径车身包络被占据，停车重规划; lookahead=%.2f m, path_pose=(%.3f,%.3f), path_yaw=%.3f rad, robot_yaw=%.3f rad, obstacle=(%.3f,%.3f,%.3f)",
          obstacle_replan_lookahead_,sample[0],sample[1],sample[3],yaw,p.x,p.y,p.z);
        return true;
      }
    }
  }
  return false;
}

void Local_Planner::getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){

  geometry_msgs::msg::PoseArray accepted_pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;
  
  rejected_trajectories_.clear();
  accepted_trajectories_.clear();

  #ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, diff_t;
  gettimeofday(&start, NULL);
  #endif
  
  //@ TODO make this omp version
  for(auto traj_it=trajectories_->begin();traj_it!=trajectories_->end();traj_it++){

    if (traj_it->getPosesSize() < 2) {
      traj_it->cost_ = -4.0;
      traj_it->rejected_by_ = "generation_empty_or_short";
      rejected_trajectories_[traj_it->rejected_by_].push_back(*traj_it);
      continue;
    }
    mpc_critics_ros_->scoreTrajectory(traj_gen_name, (*traj_it));
    
    if((*traj_it).cost_>=0){
      trajectory2posearray_cuboids((*traj_it), accepted_pose_arr, cuboids_pcl);
      accepted_trajectories_.push_back(*traj_it);
    }

    rejected_trajectories_[(*traj_it).rejected_by_].push_back(*traj_it);
    
  }

  #ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  diff_t = end_t - start_t;
  RCLCPP_WARN(this->get_logger(), "Scoring time: %.9f", diff_t);
  #endif

  std::string rejection_report;
  for (const auto& report : rejected_trajectories_) {
    if (report.first == "pass" || report.second.empty()) {
      continue;
    }
    if (!rejection_report.empty()) {
      rejection_report += ", ";
    }
    rejection_report += report.first + "=" + std::to_string(report.second.size());
  }

  if (accepted_trajectories_.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 5000,
      "Trajectory scoring rejected all candidates: generator=%s, total=%zu, "
      "rejected_by={%s}, perception_points=%zu, prune_plan_points=%zu",
      traj_gen_name.c_str(), trajectories_->size(), rejection_report.c_str(),
      perception_3d_ros_->getSharedDataPtr()->aggregate_observation_->size(),
      pcl_prune_plan_.size());
  } else {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger().get_child(name_), *clock_, 5000,
      "Trajectory scoring: generator=%s, total=%zu, accepted=%zu, "
      "rejected_by={%s}",
      traj_gen_name.c_str(), trajectories_->size(), accepted_trajectories_.size(),
      rejection_report.c_str());
  }

  trajectory_generators_ros_->expertScoring(traj_gen_name, accepted_trajectories_, rejected_trajectories_, best_traj);

  accepted_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  accepted_pose_arr.header.stamp = clock_->now();
  pub_accepted_trajectory_pose_array_->publish(accepted_pose_arr);

  geometry_msgs::msg::PoseArray best_pose_arr;
  trajectory2posearray_cuboids(best_traj, best_pose_arr, cuboids_pcl);
  best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  best_pose_arr.header.stamp = clock_->now();
  pub_best_trajectory_pose_->publish(best_pose_arr);

  // A complete snapshot with explicit boundaries; never concatenate trajectories.
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  int marker_id = 0;
  auto append_marker = [&](const base_trajectory::Trajectory& trajectory,
                           const std::string& kind) {
    if (trajectory.getPosesSize() < 2) return;
    visualization_msgs::msg::Marker marker;
    marker.header = best_pose_arr.header;
    marker.ns = kind;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = kind == "best" ? 0.035 : 0.01;
    marker.color.a = kind == "best" ? 1.0 : 0.4;
    marker.color.r = kind == "accepted" ? 0.2 : 1.0;
    marker.color.g = kind == "rejected" ? 0.3 : 0.8;
    marker.color.b = kind == "accepted" ? 1.0 : 0.2;
    marker.lifetime.sec = 1;
    // Rotation has a stationary center: draw the body-front point's swept arc
    // rather than an invisible stack of identical center positions.
    const bool rotation = std::abs(trajectory.thetav_) > 1e-6 &&
      std::hypot(trajectory.xv_, trajectory.yv_) < 1e-6;
    if (rotation) marker.ns += "_rotation";
    for (unsigned int i = 0; i < trajectory.getPosesSize(); ++i) {
      const auto& pose = trajectory.getPose(i).pose;
      auto point = pose.position;
      if (rotation) {
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w);
        const auto front = tf2::quatRotate(q, tf2::Vector3(0.45, 0, 0));
        point.x += front.x(); point.y += front.y(); point.z += front.z();
      }
      marker.points.push_back(point);
    }
    markers.markers.push_back(marker);
  };
  for (const auto& trajectory : *trajectories_)
    append_marker(trajectory, trajectory.cost_ >= 0 ? "accepted" : "rejected");
  if (best_traj.cost_ >= 0) append_marker(best_traj, "best");
  pub_local_trajectory_markers_->publish(markers);


}

// Read-only motion admission: no trajectory generator, axis state, or commands.
dddmr_sys_core::PlannerState Local_Planner::checkPathBeforeAlignment() {
  if (!got_odom_) return dddmr_sys_core::TF_FAIL;
  const auto now=clock_->now();
  const auto odom_stamp=rclcpp::Time(robot_state_.header.stamp);
  const auto pose_stamp=rclcpp::Time(trans_gbl2b_.header.stamp);
  if (!dddmr_sys_core::motionTimestampFresh(odom_stamp.nanoseconds(),(now-odom_stamp).seconds()) ||
      !dddmr_sys_core::motionTimestampFresh(pose_stamp.nanoseconds(),(now-pose_stamp).seconds()))
    return dddmr_sys_core::TF_FAIL;
  auto perception=perception_3d_ros_->getStackedPerception();
  std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(*perception->getMutex());
  if (!perception->isSensorOK()) return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  if (global_plan_.size()<3) return dddmr_sys_core::PRUNE_PLAN_FAIL;
  for (const auto& p:global_plan_)
    if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) ||
        !std::isfinite(p.pose.position.z)) return dddmr_sys_core::PRUNE_PLAN_FAIL;
  perception->aggregateObservations();
  auto observations=perception_3d_ros_->getSharedDataPtr()->aggregate_observation_;
  if (observations->size()<5) return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(
    *mpc_critics_ros_->getStackedScoringModelPtr()->getMutex());
  auto data=mpc_critics_ros_->getSharedDataPtr();
  data->robot_pose_=trans_gbl2b_;
  data->robot_state_=robot_state_;
  data->pcl_perception_=observations;
  // updateSharedData also builds the obstacle KD-tree used by the path sweep.
  data->prune_plan_=prune_plan_;
  mpc_critics_ros_->updateSharedData();
  return forwardPathBlocked() ? dddmr_sys_core::PATH_BLOCKED_REPLANNING :
                               dddmr_sys_core::TRAJECTORY_FOUND;
}

dddmr_sys_core::PlannerState Local_Planner::computeVelocityCommand(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){
  
  if(!got_odom_){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Odom is not received.");
    return dddmr_sys_core::TF_FAIL;
  }

  if(!perception_3d_ros_->getStackedPerception()->isSensorOK()){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Perception 3D is not ok.");
    return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  }
  
  if(!trajectory_generators_ros_->theoryExists(traj_gen_name)){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Specified trajectory generator: %s is not declare in yaml nor not consistent", traj_gen_name.c_str());
    return dddmr_sys_core::CONFIGURATION_ERROR;
  }

  //for timing that gives real time even in simulation
  control_loop_time_ = clock_->now();

  std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(*(perception_3d_ros_->getStackedPerception()->getMutex()));
  
  //@ update current observation for scoring
  //@ we need to visualized this for debug/justification
  perception_3d_ros_->getStackedPerception()->aggregateObservations();

  //@ forward_prune_/backward_prune_: should adapt to vehicle speed.
  //@ prune plan are used by trajectory_generators/perception
  //@ prune plan has to come after mutex lock, because global_plan_ros_sub_ reset global plan kd tree
  prunePlan(std::max(forward_prune_, heading_tracking_distance_), backward_prune_);

  sensor_msgs::msg::PointCloud2 ros2_aggregate_onservation;
  pcl::toROSMsg(*(perception_3d_ros_->getSharedDataPtr()->aggregate_observation_), ros2_aggregate_onservation);
  pub_aggregate_observation_->publish(ros2_aggregate_onservation);
  if((clock_->now()-trans_gbl2b_.header.stamp).seconds() > 2.0){
    //RCLCPP_ERROR(this->get_logger().get_child(name_), "TF out of date in local planner, the local planner wont go further.");
    //return dddmr_sys_core::TF_FAIL;
  }

  //@ TODO: Compute cuboid of each pose and send to determineIsPathBlock
  perception_3d_ros_->getSharedDataPtr()->pcl_prune_plan_ = pcl_prune_plan_;
  //perception_3d_ros_->getStackedPerception()->determineIsPathBlock(pcl_prune_plan_);

  if((clock_->now()-last_valid_prune_plan_).seconds()>=prune_plane_timeout_){
    RCLCPP_FATAL(this->get_logger().get_child(name_), "Deviate global plan too much, computeVelocityCommand() returns false.");
    return dddmr_sys_core::PRUNE_PLAN_FAIL;
  }

  //Do not create a function to set the parameters unless a nice structure is found
  //Below assignment of variables is useful when migrate to ROS2
  trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  trajectory_generators_ros_->getSharedDataPtr()->ackermann_drive_state_ = ackermann_drive_state_;
  trajectory_generators_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  trajectory_generators_ros_->getSharedDataPtr()->path_heading_lookahead_ = heading_tracking_distance_;
  //@ change max speed from perception shared data framework
  trajectory_generators_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_ 
                  = perception_3d_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_;

  traj_shared_data_ = trajectory_generators_ros_->getSharedDataPtr();
  trajectory_generators_ros_->getSharedDataPtr()->rotation_error_ =
    mpc_critics_ros_->getSharedDataPtr()->heading_deviation_;
  const double alignment_error = traj_shared_data_->rotation_error_;
  trajectory_generators_ros_->initializeTheories_wi_Shared_data();
  if (traj_gen_name != "omni_drive_simple")
    traj_shared_data_->rotation_error_ = alignment_error;

  geometry_msgs::msg::PoseArray pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;

  trajectories_ = std::make_shared<std::vector<base_trajectory::Trajectory>>();

  #ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, diff_t;
  gettimeofday(&start, NULL);
  #endif
  
  //@ We queue all trajectories in trajectories_, then score them one by one in getBestTrajectory()
  //@ single thread validaed at 0.0004 seconds with 50 samples and 0.04 with 5000 samples
  //@ omp validated at 0.003 seconds with 50 samples and 0.014 with 5000 samples
  //@ omp can only be used when samples are large
  trajectory_generators_ros_->generateAllTrajectories(traj_gen_name, trajectories_);
  for (auto& a_traj : *trajectories_) {
    if(a_traj.getPosesSize()>1)
      trajectory2posearray_cuboids(a_traj, pose_arr, cuboids_pcl);
  }
  
  #ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  diff_t = end_t - start_t;
  RCLCPP_WARN(this->get_logger(), "Trajectory generation time: %.9f", diff_t);
  #endif

  pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  pose_arr.header.stamp = clock_->now();
  pub_trajectory_pose_array_->publish(pose_arr);

  
  //cuboids_pcl.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  //pub_cuboids_.publish(cuboids_pcl);
  

  //@Update data for critics
  std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(*(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
  //@ unless we come up with a better strcuture
  //@ keep below for easy migration for ROS2
  mpc_critics_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  mpc_critics_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  mpc_critics_ros_->getSharedDataPtr()->ackermann_drive_state_ = ackermann_drive_state_;
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_ = perception_3d_ros_->getSharedDataPtr()->aggregate_observation_;
  mpc_critics_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  //@ Below function transform prune_plane from nav::msg to pcl type
  //@ Below function generate kd-tree using aggregate observation
  mpc_critics_ros_->updateSharedData();
  getBestTrajectory(traj_gen_name, best_traj);
  if (traj_gen_name == "omni_drive_simple" && obstacle_replan_lookahead_>0) {
    bool collision_rejected_motion=false;
    for (const auto& entry : rejected_trajectories_) {
      if (entry.first.find("collision")==std::string::npos) continue;
      for (const auto& t : entry.second)
        collision_rejected_motion |= !trajectory_generators::zeroCommand(t.xv_,t.yv_,t.thetav_);
    }
    const bool brake_only=best_traj.cost_<0 ||
      trajectory_generators::zeroCommand(best_traj.xv_,best_traj.yv_,best_traj.thetav_);
    if (forwardPathBlocked() || (collision_rejected_motion && brake_only)) {
      RCLCPP_WARN_THROTTLE(get_logger(),*clock_,1000,
        "停车原因：前方路径阻塞或仅剩刹车候选，立即请求新路径");
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
    }
  }

  auto t_diff = clock_->now() - control_loop_time_;
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Full control cycle time: %.9f", t_diff.seconds());

  if(t_diff.seconds() > 1./controller_frequency_){
    RCLCPP_WARN(this->get_logger().get_child(name_), "Local planner control time exceed expect time: %.2f but is %.2f", 1./controller_frequency_, t_diff.seconds());
  }
  
  //@Loop opinions
  std::vector<perception_3d::PerceptionOpinion> opinions = perception_3d_ros_->getStackedPerception()->getOpinions();
  for(auto opinion_it=opinions.begin(); opinion_it!=opinions.end();opinion_it++){
    if((*opinion_it)==perception_3d::PATH_BLOCKED_WAIT){
      RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "Found the prune plan is blocked, go to wait state.");
      return dddmr_sys_core::PATH_BLOCKED_WAIT;
    }
    else if((*opinion_it)==perception_3d::PATH_BLOCKED_REPLANNING){
      RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "Found the prune plan is blocked, go to replanning.");
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;      
    }
  }


  if(best_traj.cost_<0){
    RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "All trajectories are rejected by critics.");
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }
  else{
    return dddmr_sys_core::TRAJECTORY_FOUND;
  }
  
  //@ Reset kd tree/observations because it is shared_ptr and copied from perception_ros
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
}

void Local_Planner::trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr,
                                      pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl){

  for(unsigned int i=0;i<a_traj.getPosesSize();i++){
      auto p = a_traj.getPose(i);
      pose_arr.poses.push_back(p.pose);
      //@ For cuboids debug
      //cuboids_pcl += a_traj.getCuboid(i);       
  }

}

/*
void Local_Planner::cbMCL_ground_normal(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  
  ground_with_normals_.reset(new pcl::PointCloud<pcl::PointNormal>);
  pcl::fromROSMsg(*msg, *ground_with_normals_);
  if(perception_3d_ros_->getGlobalUtils()->getGblFrame().compare(msg->header.frame_id) != 0)
    ROS_ERROR("%s: the global frame is not consistent with topics and perception setting.", name_.c_str());
  global_frame_ = msg->header.frame_id;
  normal2quaternion();
}

void Local_Planner::normal2quaternion(){

  visualization_msgs::MarkerArray markerArray;
  for(size_t i=0;i<ground_with_normals_->points.size();i++){

    tf2::Vector3 axis_vector(ground_with_normals_->points[i].normal_x, ground_with_normals_->points[i].normal_y, ground_with_normals_->points[i].normal_z);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    //@Create arrow
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = ground_with_normals_->header.frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = ground_with_normals_->points[i].x;
    marker.pose.position.y = ground_with_normals_->points[i].y;
    marker.pose.position.z = ground_with_normals_->points[i].z;
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3; //scale.x is the arrow length,
    marker.scale.y = 0.05; //scale.y is the arrow width 
    marker.scale.z = 0.1; //scale.z is the arrow height. 

    double angle = atan2(ground_with_normals_->points[i].normal_z, 
                  sqrt(ground_with_normals_->points[i].normal_x*ground_with_normals_->points[i].normal_x+ ground_with_normals_->points[i].normal_y*ground_with_normals_->points[i].normal_y) ) * 180 / 3.1415926535;

    if(fabs(angle)<=10){
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.0f;      
    }
    else{
      marker.color.r = 0.0f;
      marker.color.g = 0.8f;
      marker.color.b = 0.2f; 
    }

    marker.color.a = 0.6f;   
    markerArray.markers.push_back(marker); 
  }
  pub_pc_normal_.publish(markerArray);
}
*/

}// end of name space
