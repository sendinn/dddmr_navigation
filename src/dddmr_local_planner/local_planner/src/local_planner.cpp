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

  tracking_trajectory_generator_ = declare_parameter<std::string>(
    "tracking_trajectory_generator", "omni_drive_simple");
  if (tracking_trajectory_generator_.empty())
    throw std::invalid_argument("tracking_trajectory_generator cannot be empty");

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

// 接收并保存任务管理器给出的最新全局路径，为后续局部路径裁剪建立空间索引。
// 本函数只替换局部规划器的参考路径，不生成速度轨迹；轨迹生成和评分在
// computeVelocityCommand() 中进行，prunePlan() 会使用这里建立的 KD-tree。
void Local_Planner::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan) {

  // 局部规划至少需要三个路径点才能可靠确定前后关系和行驶方向。
  // 路径不足时直接拒绝本次更新，保留原有 global_plan_ 和 KD-tree。
  if(orig_global_plan.size()<3){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Size of global plan is smaller than 3.");
    return;
  }

  // 保存完整的 PoseStamped 路径。位姿中的位置供裁剪和距离判断使用，
  // 朝向及末点位姿仍保留在 global_plan_ 中，供方向跟踪和终点判断使用。
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // 将 ROS 路径转换为只包含 XYZ 的 PCL 点云，点的顺序和 global_plan_ 完全一致。
  // 因此 KD-tree 返回的点索引可以直接作为 global_plan_ 的数组索引使用。
  pcl_global_plan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto gbl_it = global_plan_.begin(); gbl_it!=global_plan_.end();gbl_it++){
    pcl::PointXYZ pt;
    pt.x = (*gbl_it).pose.position.x;
    pt.y = (*gbl_it).pose.position.y;
    pt.z = (*gbl_it).pose.position.z;
    pcl_global_plan_->push_back(pt);
  }

  // 为最新全局路径重建 KD-tree。prunePlan() 用它查找离机器人最近的路径点，
  // 再以该点为中心截取前后一定距离的 prune_plan_，交给轨迹生成器和评分器。
  kdtree_global_plan_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kdtree_global_plan_->setInputCloud (pcl_global_plan_);
  // 控制状态下可能重复收到同一路径，日志限流为每 10 秒最多输出一次。
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

// 以机器人在完整全局路径上的最近点为中心，截取前后指定弧长的局部参考路径。
// forward_distance/backward_distance 是沿路径逐段累计的距离，不是以机器人为圆心的半径。
// prune_plan_ 保留 PoseStamped 供轨迹生成和评分；pcl_prune_plan_ 额外用 intensity
// 标记前后方向，供 perception_3d 的路径阻塞策略使用。
void Local_Planner::prunePlan(double forward_distance, double backward_distance){

  // 每次控制周期重新生成裁剪结果，避免上一周期的局部路径残留。
  prune_plan_.poses.clear();
  pcl_prune_plan_.clear();
  // setPlan() 正常只接受至少三个点；这里再次防御空路径或尚未完成初始化的情况。
  // 提前返回时不会刷新 last_valid_prune_plan_，持续失败最终会触发 PRUNE_PLAN_FAIL。
  if(pcl_global_plan_->points.size()<3)
    return;

  // nearestKSearch() 的输出：最近点在 pcl_global_plan_ 中的索引，以及三维距离平方。
  // setPlan() 保持 PCL 点与 global_plan_ 的顺序一致，因此该索引可直接访问 global_plan_。
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  // KD-tree 查询点使用当前 map -> base_link 变换中的机器人全局 XYZ。
  pcl::PointXYZ robot_pose;
  robot_pose.x = trans_gbl2b_.transform.translation.x;
  robot_pose.y = trans_gbl2b_.transform.translation.y;
  robot_pose.z = trans_gbl2b_.transform.translation.z;

  // 找不到最近路径点时无法确定裁剪中心，本轮不生成 prune_plan_。
  if ( kdtree_global_plan_->nearestKSearch (robot_pose, 1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0 ){
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "Ready to fix some exception here.");
    return;
  }


  // 机器人到最近全局路径点的三维距离超过 1 m 时认为已经严重偏离路线。
  // 本轮保持裁剪结果为空；连续超过 prune_plane_timeout_ 后由上层停车并重新规划。
  if(sqrt(pointNKNSquaredDistance[0])>1.0){
    RCLCPP_DEBUG(this->get_logger().get_child(name_), "Deviate from plan, fix some exception here.");
    //@ consider to clear prune_plan in model_shared_data?
    return;
  }

  // ---------------- 向路径起点方向截取 ----------------
  // 从最近点开始按索引递减，逐段扣减 backward_distance。
  geometry_msgs::msg::PoseStamped last_pose = global_plan_[pointIdxNKNSearch[0]];
  for(int i=pointIdxNKNSearch[0]; i>=0; i--){
    prune_plan_.poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x; pt.y = global_plan_[i].pose.position.y; pt.z = global_plan_[i].pose.position.z;
    // intensity=-1 不是激光强度，而是“机器人后方路径点”的内部标签。
    pt.intensity = -1;
    pcl_prune_plan_.points.push_back(pt);
    if(i<pointIdxNKNSearch[0]){
      // 按相邻路径点的三维线段长度累计，而不是按索引数量截取。
      backward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    // 当前点已经加入后才判断，因此末点可能比请求距离多覆盖一个路径线段。
    if(backward_distance<0)
      break;
  }
  
  // 上面的插入顺序是“最近点 -> 更早的点”，与正常行驶顺序相反；
  // 这里只反转 ROS 路径，使 prune_plan_ 恢复为“较早点 -> 最近点”的正向顺序。
  std::reverse(prune_plan_.poses.begin(),prune_plan_.poses.end()); 

  // ---------------- 向路径终点方向截取 ----------------
  // 从最近点开始按索引递增。最近点会在前后两段的交界处再次加入，
  // 后续评分按有序局部参考线继续处理这些 PoseStamped。
  for(int i=pointIdxNKNSearch[0];i<global_plan_.size();i++){
    prune_plan_.poses.push_back(global_plan_[i]);
    pcl::PointXYZI pt;
    pt.x = global_plan_[i].pose.position.x; pt.y = global_plan_[i].pose.position.y; pt.z = global_plan_[i].pose.position.z;
    // PCL 路径用非负 intensity 标记前向部分：全局路径第 0 点为 0，其余前向点为 1。
    if(i == 0){
      pt.intensity = 0;
    }
    else{
      pt.intensity = 1;
    }
    pcl_prune_plan_.points.push_back(pt);

    if(i>pointIdxNKNSearch[0]){
      // 从最近点开始累计相邻路径点的三维线段长度。
      forward_distance -= getDistanceBTWPoseStamp(last_pose, global_plan_[i]);
    }
    last_pose = global_plan_[i];
    // 与后向截取相同，超出阈值的当前点已经保留，保证局部段覆盖请求距离。
    if(forward_distance<0)
      break;
  }
  
  // 裁剪路径仍位于全局规划坐标系；时间戳表示本轮局部路径生成时间。
  prune_plan_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  prune_plan_.header.stamp = clock_->now();
  // 发布 /prune_plan 供调试显示，并记录本轮最近点查询和裁剪成功。
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
    bool rotation = std::abs(trajectory.thetav_) > 1e-6 &&
      std::hypot(trajectory.xv_, trajectory.yv_) < 1e-6;
    // 分阶段候选可能先发 yaw、随后直行。只给中心始终静止的纯旋转画机头弧线，
    // 其余轨迹显示真实中心线，避免把先转后走误画为整段机头圆弧。
    if (rotation) {
      const auto first=trajectory.getPose(0).pose.position;
      for (unsigned int i=1;i<trajectory.getPosesSize();++i) {
        const auto point=trajectory.getPose(i).pose.position;
        if (std::hypot(point.x-first.x,point.y-first.y)>0.005) {rotation=false;break;}
      }
    }
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

// 根据最新机器人状态、局部参考路径和实时障碍生成候选轨迹，并选出本控制周期的最优轨迹。
// traj_gen_name 是配置实例名；跟踪实例由 tracking_trajectory_generator 指定，
// 起步/终点对向实例由任务层的 heading_trajectory_generator 指定。
// 成功时 best_traj 保存选中轨迹及其 vx/vy/wz；返回值说明成功、感知/TF 异常、
// 路径阻塞、路径裁剪失败或全部候选被拒绝，实际发布 /cmd_vel 由 p2p_move_base 完成。
dddmr_sys_core::PlannerState Local_Planner::computeVelocityCommand(std::string traj_gen_name, base_trajectory::Trajectory& best_traj, bool rotation_test, double test_heading){
  traj_shared_data_=trajectory_generators_ros_->getSharedDataPtr();
  rotation_test_diagnostics_.clear();
  traj_shared_data_->rotation_test_=rotation_test;
  traj_shared_data_->rotation_test_heading_=test_heading;
  traj_shared_data_->rotation_test_supported_=false;
  traj_shared_data_->rotation_test_complete_=false;
  
  // 没有里程计就无法从当前实测速度预测未来运动，也不能安全生成控制指令。
  if(!got_odom_){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Odom is not received.");
    return dddmr_sys_core::TF_FAIL;
  }

  // 感知插件数据缺失或过期时停止本轮规划，避免使用陈旧障碍生成运动轨迹。
  if(!perception_3d_ros_->getStackedPerception()->isSensorOK()){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Perception 3D is not ok.");
    return dddmr_sys_core::PERCEPTION_MALFUNCTION;
  }
  
  // 生成器名称必须已在 YAML 的 trajectory_generators.plugins 中加载。
  if(!trajectory_generators_ros_->theoryExists(traj_gen_name)){
    RCLCPP_ERROR(this->get_logger().get_child(name_), "Specified trajectory generator: %s is not declare in yaml nor not consistent", traj_gen_name.c_str());
    return dddmr_sys_core::CONFIGURATION_ERROR;
  }

  // 记录本控制周期开始时间，用于检查局部规划是否超过 controller_frequency_ 对应的周期。
  control_loop_time_ = clock_->now();

  // 锁住感知共享数据，使障碍汇总、路径裁剪、轨迹生成和评分使用同一轮数据视图。
  std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(*(perception_3d_ros_->getStackedPerception()->getMutex()));
  
  // 汇总所有感知插件的 observation 点，后续碰撞评分使用这份实时障碍点云。
  perception_3d_ros_->getStackedPerception()->aggregateObservations();

  // 从完整 global_plan_ 中截取机器人附近的局部参考段 prune_plan_。
  // 前向范围至少覆盖 heading_tracking_distance_，确保方向计算有足够的前视路径；
  // 后向范围保留机器人身后的少量路径，便于最近点和偏差判断。
  if (rotation_test) {
    // Scoring reference only; no forward-motion target or global replan.
    prune_plan_.poses.clear(); pcl_prune_plan_.clear();
    prune_plan_.header=trans_gbl2b_.header;
    for (int i=0;i<3;++i) {
      geometry_msgs::msg::PoseStamped p;p.header=prune_plan_.header;
      p.pose.position.x=trans_gbl2b_.transform.translation.x+0.1*i*std::cos(test_heading);
      p.pose.position.y=trans_gbl2b_.transform.translation.y+0.1*i*std::sin(test_heading);
      p.pose.position.z=trans_gbl2b_.transform.translation.z;
      p.pose.orientation.z=std::sin(test_heading/2);p.pose.orientation.w=std::cos(test_heading/2);
      prune_plan_.poses.push_back(p);
    }
    last_valid_prune_plan_=clock_->now();
  } else {
    prunePlan(std::max(forward_prune_, heading_tracking_distance_), backward_prune_);
  }

  // 发布本轮聚合后的障碍点云，仅用于显示和诊断，不参与额外计算。
  sensor_msgs::msg::PointCloud2 ros2_aggregate_onservation;
  pcl::toROSMsg(*(perception_3d_ros_->getSharedDataPtr()->aggregate_observation_), ros2_aggregate_onservation);
  pub_aggregate_observation_->publish(ros2_aggregate_onservation);
  // 当前 TF 超时处理已被注释，因此此条件即使满足也不会中止本轮规划。
  if((clock_->now()-trans_gbl2b_.header.stamp).seconds() > 2.0){
    //RCLCPP_ERROR(this->get_logger().get_child(name_), "TF out of date in local planner, the local planner wont go further.");
    //return dddmr_sys_core::TF_FAIL;
  }

  // 将裁剪路径的 PCL 表示共享给 perception_3d，供路径阻塞相关插件使用。
  // 当前 determineIsPathBlock() 的直接调用仍被注释，具体阻塞意见由已启用插件产生。
  perception_3d_ros_->getSharedDataPtr()->pcl_prune_plan_ = pcl_prune_plan_;
  //perception_3d_ros_->getStackedPerception()->determineIsPathBlock(pcl_prune_plan_);

  // prunePlan() 长时间无法得到有效局部参考段，通常表示机器人离全局路径过远、
  // 路径为空或最近点查询失败。返回后由任务状态机停车并请求重新规划。
  if((clock_->now()-last_valid_prune_plan_).seconds()>=prune_plane_timeout_){
    RCLCPP_FATAL(this->get_logger().get_child(name_), "Deviate global plan too much, computeVelocityCommand() returns false.");
    return dddmr_sys_core::PRUNE_PLAN_FAIL;
  }

  // 向轨迹生成器提供本周期输入：当前地图位姿、实测底盘状态、裁剪路径、
  // 航向前视距离，以及感知层根据障碍计算出的当前允许最大线速度。
  trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  trajectory_generators_ros_->getSharedDataPtr()->ackermann_drive_state_ = ackermann_drive_state_;
  trajectory_generators_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  trajectory_generators_ros_->getSharedDataPtr()->path_heading_lookahead_ = heading_tracking_distance_;
  //@ change max speed from perception shared data framework
  trajectory_generators_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_ 
                  = perception_3d_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_;

  // 把评分层计算的航向偏差传给旋转类生成器。initializeTheories_wi_Shared_data()
  // 会依据最新共享数据重置采样状态，并准备本轮可采样的速度组合。
  traj_shared_data_ = trajectory_generators_ros_->getSharedDataPtr();
  trajectory_generators_ros_->getSharedDataPtr()->rotation_error_ =
    mpc_critics_ros_->getSharedDataPtr()->heading_deviation_;
  const double alignment_error = traj_shared_data_->rotation_error_;
  trajectory_generators_ros_->initializeTheories_wi_Shared_data();
  // 非正常行驶生成器继续使用初始化前的对向误差，避免初始化过程覆盖旋转目标。
  if (traj_gen_name != tracking_trajectory_generator_)
    traj_shared_data_->rotation_error_ = alignment_error;

  if (rotation_test && !traj_shared_data_->rotation_test_supported_)
    return dddmr_sys_core::CONFIGURATION_ERROR;

  // pose_arr 汇总所有候选轨迹的预测位姿，用于可视化；cuboids_pcl 当前未发布。
  geometry_msgs::msg::PoseArray pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;

  // 每个控制周期重新创建候选容器，防止上一周期的轨迹混入本轮评分。
  trajectories_ = std::make_shared<std::vector<base_trajectory::Trajectory>>();

  #ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, diff_t;
  gettimeofday(&start, NULL);
  #endif
  
  // 按生成器的速度采样表生成全部未来轨迹。每条轨迹包含候选 vx/vy/wz，
  // 以及在模拟时域内按运动模型积分得到的一系列预测位姿。
  trajectory_generators_ros_->generateAllTrajectories(traj_gen_name, trajectories_);
  // 将所有有效候选的预测位姿拼入调试消息；这不是最终执行路径。
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
  // trajectory 话题包含全部生成候选，仅供观察；最终选择结果写入 best_traj。
  pub_trajectory_pose_array_->publish(pose_arr);

  
  //cuboids_pcl.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  //pub_cuboids_.publish(cuboids_pcl);
  

  // 锁住评分器并写入本轮统一输入。评分器会把 observation 转为碰撞查询结构，
  // 并结合 prune_plan_ 对候选轨迹执行碰撞、贴线、前视目标等评分。
  std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(*(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
  mpc_critics_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b_;
  mpc_critics_ros_->getSharedDataPtr()->robot_state_ = robot_state_;
  mpc_critics_ros_->getSharedDataPtr()->ackermann_drive_state_ = ackermann_drive_state_;
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_ = perception_3d_ros_->getSharedDataPtr()->aggregate_observation_;
  mpc_critics_ros_->getSharedDataPtr()->prune_plan_ = prune_plan_;
  // 更新评分共享数据，包括路径/障碍的 PCL 表示及相关 KD-tree。
  mpc_critics_ros_->updateSharedData();
  // 逐条评分：被任一硬约束拒绝的轨迹记为无效，其余轨迹中最低代价者写入 best_traj。
  getBestTrajectory(traj_gen_name, best_traj);
  if(rotation_test) {
    rotation_test_diagnostics_="generated="+std::to_string(trajectories_->size())+
      " accepted="+std::to_string(accepted_trajectories_.size());
    for(const auto& entry:rejected_trajectories_)
      rotation_test_diagnostics_+=" rejected["+entry.first+"]="+std::to_string(entry.second.size());
  }
  // 正常行驶时增加提前重规划判断：前方参考路径已经阻塞，或者运动候选均因碰撞
  // 被拒绝而只剩刹车/无有效轨迹时，要求任务状态机立即停车并刷新全局路径。
  if (!rotation_test && traj_gen_name == tracking_trajectory_generator_ && obstacle_replan_lookahead_>0) {
    bool collision_rejected_motion=false;
    for (const auto& entry : rejected_trajectories_) {
      if (entry.first.find("collision")==std::string::npos) continue;
      for (const auto& t : entry.second)
        collision_rejected_motion |= !trajectory_generators::zeroCommand(t.xv_,t.yv_,t.thetav_);
    }
    const bool brake_only=best_traj.cost_<0 ||
      trajectory_generators::zeroCommand(best_traj.xv_,best_traj.yv_,best_traj.thetav_);
    const bool reference_path_blocked = forwardPathBlocked();
    const auto trajectory_state = trajectory_generators_ros_->getSharedDataPtr();
    const bool safe_single_axis_avoidance =
      trajectory_state->single_axis_avoidance_active_ &&
      trajectory_state->single_axis_avoidance_has_safe_command_;
    if ((reference_path_blocked || (collision_rejected_motion && brake_only)) &&
        !safe_single_axis_avoidance) {
      RCLCPP_WARN_THROTTLE(get_logger(),*clock_,1000,
        "停车原因：前方路径阻塞或仅剩刹车候选，立即请求新路径");
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;
    }
    if (reference_path_blocked && safe_single_axis_avoidance) {
      RCLCPP_INFO_THROTTLE(get_logger(),*clock_,1000,
        "参考路径仍被占据；已有安全单轴避障候选，保持停车切换或执行避障轴");
    }
  }

  if (rotation_test && (best_traj.cost_<0 ||
      trajectory_generators::zeroCommand(best_traj.xv_,best_traj.yv_,best_traj.thetav_))) {
    for (const auto& entry:rejected_trajectories_)
      for (const auto& t:entry.second)
        if (std::abs(t.thetav_)>1e-6) {
          RCLCPP_WARN(get_logger(), "Rotation test rejected by critic: %s",entry.first.c_str());
          return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
        }
  }
  auto t_diff = clock_->now() - control_loop_time_;
  RCLCPP_DEBUG(this->get_logger().get_child(name_), "Full control cycle time: %.9f", t_diff.seconds());

  // 规划耗时超过一个控制周期时只记录警告，本轮结果仍继续按下方状态返回。
  if(t_diff.seconds() > 1./controller_frequency_){
    RCLCPP_WARN(this->get_logger().get_child(name_), "Local planner control time exceed expect time: %.2f but is %.2f", 1./controller_frequency_, t_diff.seconds());
  }
  
  // 感知插件还可以给出路径阻塞意见：WAIT 表示保持等待，REPLANNING 表示刷新全局路径。
  std::vector<perception_3d::PerceptionOpinion> opinions = perception_3d_ros_->getStackedPerception()->getOpinions();
  const auto trajectory_state = trajectory_generators_ros_->getSharedDataPtr();
  const bool safe_single_axis_avoidance =
    traj_gen_name == tracking_trajectory_generator_ &&
    trajectory_state->single_axis_avoidance_active_ &&
    trajectory_state->single_axis_avoidance_has_safe_command_;
  for(auto opinion_it=opinions.begin(); opinion_it!=opinions.end();opinion_it++){
    if (safe_single_axis_avoidance &&
        ((*opinion_it)==perception_3d::PATH_BLOCKED_WAIT ||
         (*opinion_it)==perception_3d::PATH_BLOCKED_REPLANNING)) {
      RCLCPP_INFO_THROTTLE(this->get_logger().get_child(name_), *clock_, 1000,
        "感知报告参考路径阻塞；已有安全单轴避障候选，本轮继续局部避障");
      continue;
    }
    if((*opinion_it)==perception_3d::PATH_BLOCKED_WAIT){
      RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "Found the prune plan is blocked, go to wait state.");
      return dddmr_sys_core::PATH_BLOCKED_WAIT;
    }
    else if((*opinion_it)==perception_3d::PATH_BLOCKED_REPLANNING){
      RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "Found the prune plan is blocked, go to replanning.");
      return dddmr_sys_core::PATH_BLOCKED_REPLANNING;      
    }
  }


  // cost_ < 0 表示所有候选都被评分器拒绝；非负则 best_traj 可由上层转换为 /cmd_vel。
  if(best_traj.cost_<0){
    RCLCPP_WARN_THROTTLE(this->get_logger().get_child(name_), *clock_, 5000, "All trajectories are rejected by critics.");
    return dddmr_sys_core::ALL_TRAJECTORIES_FAIL;
  }
  else{
    return dddmr_sys_core::TRAJECTORY_FOUND;
  }
  
  // 注意：上面的两个分支都会 return，因此当前清理代码实际不可达。
  // 这里原意是断开评分器持有的感知点云/KD-tree，共享指针生命周期由现有对象管理。
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
