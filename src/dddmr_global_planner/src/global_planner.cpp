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
#include <global_planner/global_planner.h>

using namespace std::chrono_literals;

namespace global_planner
{

GlobalPlanner::GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  current_handle_.reset();
  current_handle_ = goal_handle;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d){
  
  static_ground_size_ = 0;
  perception_3d_ros_ = perception_3d;
  graph_ready_ = false;
  has_initialized_ = false;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  global_plan_result_ = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
  
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_map_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  kdtree_ground_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  
  declare_parameter("turning_weight", rclcpp::ParameterValue(0.1));
  this->get_parameter("turning_weight", turning_weight_);
  RCLCPP_INFO(this->get_logger(), "turning_weight: %.2f", turning_weight_);    

  declare_parameter("enable_detail_log", rclcpp::ParameterValue(false));
  this->get_parameter("enable_detail_log", enable_detail_log_);
  RCLCPP_INFO(this->get_logger(), "enable_detail_log: %d", enable_detail_log_);    

  declare_parameter("a_star_expanding_radius", rclcpp::ParameterValue(0.5));
  this->get_parameter("a_star_expanding_radius", a_star_expanding_radius_);
  RCLCPP_INFO(this->get_logger(), "a_star_expanding_radius: %.2f", a_star_expanding_radius_);    

  footprint_.front = declare_parameter<double>("footprint.front", 0.47);
  footprint_.back = declare_parameter<double>("footprint.back", 0.47);
  footprint_.left = declare_parameter<double>("footprint.left", 0.278);
  footprint_.right = declare_parameter<double>("footprint.right", 0.278);
  footprint_.bottom = declare_parameter<double>("footprint.bottom", 0.10);
  footprint_.top = declare_parameter<double>("footprint.top", 0.66);
  footprint_.sample_step = declare_parameter<double>("footprint.sample_step", 0.05);
  if (!std::isfinite(footprint_.front) || footprint_.front <= 0.0 ||
      !std::isfinite(footprint_.back) || footprint_.back <= 0.0 ||
      !std::isfinite(footprint_.left) || footprint_.left <= 0.0 ||
      !std::isfinite(footprint_.right) || footprint_.right <= 0.0 ||
      !std::isfinite(footprint_.bottom) || footprint_.bottom < 0.0 ||
      !std::isfinite(footprint_.top) || footprint_.top <= footprint_.bottom ||
      !std::isfinite(footprint_.sample_step) || footprint_.sample_step <= 0.0 ||
      footprint_.sample_step > 0.25) {
    throw std::invalid_argument("Invalid cuboid footprint parameters");
  }
  RCLCPP_INFO(
    this->get_logger(),
    "A* cuboid: front=%.3f back=%.3f left=%.3f right=%.3f bottom=%.3f top=%.3f, sweep_step=%.3f",
    footprint_.front, footprint_.back, footprint_.left, footprint_.right,
    footprint_.bottom, footprint_.top, footprint_.sample_step);

  declare_parameter("use_pre_graph", rclcpp::ParameterValue(false));
  this->get_parameter("use_pre_graph", use_pre_graph_);
  RCLCPP_INFO(this->get_logger(), "use_pre_graph: %d", use_pre_graph_);    

  declare_parameter("find_start_tolerance", rclcpp::ParameterValue(0.5));
  this->get_parameter("find_start_tolerance", find_start_tolerance_);
  RCLCPP_INFO(this->get_logger(), "find_start_tolerance: %.2f", find_start_tolerance_);    
  // Zero offset preserves the original spherical search for existing robots.
  start_ground_offset_ = declare_parameter<double>("start_ground_offset", 0.0);
  start_ground_z_tolerance_ = declare_parameter<double>("start_ground_z_tolerance", 0.2);
  if (!std::isfinite(start_ground_offset_) || start_ground_offset_ < 0.0 ||
      !std::isfinite(start_ground_z_tolerance_) || start_ground_z_tolerance_ <= 0.0 ||
      !std::isfinite(find_start_tolerance_) || find_start_tolerance_ <= 0.0) {
    throw std::invalid_argument("Invalid start ground projection parameters");
  }

  

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

  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;
  
  perception_3d_check_timer_ = this->create_wall_timer(500ms, std::bind(&GlobalPlanner::checkPerception3DThread, this), action_server_group_);
  
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, 
      std::bind(&GlobalPlanner::cbClickedPoint, this, std::placeholders::_1), sub_options);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  pub_static_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("static_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_weighted_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("weighted_ground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_plan",
    std::bind(&GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

GlobalPlanner::~GlobalPlanner(){

  //perception_3d_ros_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  a_star_planner_.reset();
  a_star_planner_pre_graph_.reset();
  action_server_global_planner_.reset();
  kdtree_ground_.reset();
  kdtree_map_.reset();
  pcl_ground_.reset();
  pcl_map_.reset();
}

void GlobalPlanner::checkPerception3DThread(){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    return;
  }
  
  if(static_ground_size_!=perception_3d_ros_->getSharedDataPtr()->static_ground_size_){
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    *pcl_ground_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_ground_);
    global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    *kdtree_ground_ = *(perception_3d_ros_->getSharedDataPtr()->kdtree_ground_);
    *kdtree_map_ = *(perception_3d_ros_->getSharedDataPtr()->kdtree_map_);
    *pcl_map_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_map_);
    static_graph_ = *perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_; //@ node weight
    RCLCPP_INFO(this->get_logger(), "Ground and Kd-tree ground have been received from perception_3d.");
    getStaticGraphFromPerception3D();
    static_ground_size_ = perception_3d_ros_->getSharedDataPtr()->static_ground_size_;
  }

}

void GlobalPlanner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Received clicked goal before static layer is ready");
    return;
  }

  geometry_msgs::msg::PoseStamped start, goal;

  goal.pose.position.x = clicked_goal->point.x;
  goal.pose.position.y = clicked_goal->point.y;
  goal.pose.position.z = clicked_goal->point.z;

  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to transform pointcloud: %s", e.what());
  }
  
  std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
  unsigned int start_id, goal_id;
  std::vector<unsigned int> path;
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  nav_msgs::msg::Path ros_path;

  if(getStartGoalID(start, goal, start_id, goal_id)){
    if(!use_pre_graph_)
      a_star_planner_->getPath(start_id, goal_id, path);
    else
      a_star_planner_pre_graph_->getPath(start_id, goal_id, path);
  }


  if(path.empty()){
    RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
  }
  else{
    //postSmoothPath(path, smoothed_path);
    getROSPath(path, ros_path);
    pub_path_->publish(ros_path);
    RCLCPP_INFO(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
  }

}

void GlobalPlanner::postSmoothPath(std::vector<unsigned int>& path_id, std::vector<unsigned int>& smoothed_path_id){
  
  smoothed_path_id.clear();
  geometry_msgs::msg::PoseStamped current_pst;
  current_pst.pose.position.x = pcl_ground_->points[path_id[0]].x;
  current_pst.pose.position.y = pcl_ground_->points[path_id[0]].y;
  current_pst.pose.position.z = pcl_ground_->points[path_id[0]].z;
  
  smoothed_path_id.push_back(path_id[0]);

  for(auto it=1;it<path_id.size()-1;it++){

    geometry_msgs::msg::PoseStamped next_pst;
    next_pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    next_pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    next_pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    double vx,vy,vz;
    vx = next_pst.pose.position.x - current_pst.pose.position.x;
    vy = next_pst.pose.position.y - current_pst.pose.position.y;
    vz = next_pst.pose.position.z - current_pst.pose.position.z;
    pcl::PointXYZI smooth_start;
    smooth_start.x = current_pst.pose.position.x;
    smooth_start.y = current_pst.pose.position.y;
    smooth_start.z = current_pst.pose.position.z;
    pcl::PointXYZI smooth_end;
    smooth_end.x = next_pst.pose.position.x;
    smooth_end.y = next_pst.pose.position.y;
    smooth_end.z = next_pst.pose.position.z;
    if (!isFootprintSweepClear(smooth_start, smooth_end)) {
      smoothed_path_id.push_back(path_id[it]);
      current_pst = next_pst;
      continue;
    }
    double unit = sqrt(vx*vx + vy*vy + vz*vz);
    
    tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    current_pst.pose.orientation.x = q.getX();
    current_pst.pose.orientation.y = q.getY();
    current_pst.pose.orientation.z = q.getZ();
    current_pst.pose.orientation.w = q.getW();     

    //@Interpolation to make global plan smoother and better resolution for local planner
    for(double step=0.05;step<0.99;step+=0.05){
      pcl::PointXYZI pst_inter_polate_pc;
      pst_inter_polate_pc.x = current_pst.pose.position.x + vx*step;
      pst_inter_polate_pc.y = current_pst.pose.position.y + vy*step;
      pst_inter_polate_pc.z = current_pst.pose.position.z + vz*step;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if(kdtree_ground_->radiusSearch(pst_inter_polate_pc, 1.0, pointIdxRadiusSearch, pointRadiusSquaredDistance)<2){
        //@ not on the ground
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      double dx = vx*step;
      double dy = vy*step;
      double dr = sqrt(dx*dx+dy*dy);
      double dz = fabs(vz*step);
      float vertical_angle = std::asin(dz / dr);
      if(dr>0.5 && vertical_angle>0.349){
        //@ z jump
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
      if(dr>20.0){
        //@ longer than 10 meter
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
    }
  }
  smoothed_path_id.push_back(path_id[path_id.size()-1]);
}

// 将 A* 返回的地面点索引序列转换为 nav_msgs::msg::Path。
// 每个图节点会被转换为带路径朝向的 PoseStamped，节点之间再插入中间点，
// 以提高下游局部规划器获得的全局路径分辨率。
void GlobalPlanner::getROSPath(std::vector<unsigned int>& path_id, nav_msgs::msg::Path& ros_path){

  // 整条路径使用全局规划坐标系和同一时间戳。
  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();


  // path_id 中的每个值都是 pcl_ground_->points 的索引。
  for(auto it=0;it<path_id.size();it++){
    geometry_msgs::msg::PoseStamped pst;
    pst.header = ros_path.header;
    // 把当前地面图节点的 XYZ 坐标复制到 ROS 路径位姿。
    pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    // 选择一个朝向参考点，用“当前点 -> 参考点”计算本路径点朝向。
    geometry_msgs::msg::PoseStamped next_pst;
    // path_id.size()-1 是最后一个有效索引。条件为 true 表示当前节点不是末点，
    // 因此 path_id[it+1] 存在，可以安全地用下一个节点计算当前朝向。
    // 例如共 4 个节点时，it=0、1、2 进入该分支，it=3 是末点，进入下方末点处理分支。
    if(it<path_id.size()-1){
      // 普通节点直接朝向下一个图节点。
      next_pst.pose.position.x = pcl_ground_->points[path_id[it+1]].x;
      next_pst.pose.position.y = pcl_ground_->points[path_id[it+1]].y;
      next_pst.pose.position.z = pcl_ground_->points[path_id[it+1]].z;
    } else if (it > 0) {
      // 最后一个图节点没有 next，沿“前一点 -> 当前点”向前外推一个虚拟点，
      // 使末点保持到达方向。如果默认使用 (0,0,0)，末点会错误地指向地图原点。
      const auto & previous = pcl_ground_->points[path_id[it-1]];
      next_pst.pose.position.x = 2.0 * pst.pose.position.x - previous.x;
      next_pst.pose.position.y = 2.0 * pst.pose.position.y - previous.y;
      next_pst.pose.position.z = 2.0 * pst.pose.position.z - previous.z;
    } else {
      // 路径只有一个图节点时没有可用方向，令参考点与当前点重合。
      next_pst.pose.position = pst.pose.position;
    }


    // 路径段方向向量，同时用于姿态计算和节点间插值。
    double vx,vy,vz;
    vx = next_pst.pose.position.x - pst.pose.position.x;
    vy = next_pst.pose.position.y - pst.pose.position.y;
    vz = next_pst.pose.position.z - pst.pose.position.z;

    if(vz!=0){
      // 三维路径段：将车体的 +X 前向对齐路径段三维方向。
      double unit = sqrt(vx*vx + vy*vy + vz*vz);
      
      // 路径段的单位方向向量。
      tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

      // 以 +X 为车体默认前向，计算旋转轴和夹角后构造四元数。
      tf2::Vector3 up_vector(1.0, 0.0, 0.0);
      tf2::Vector3 right_vector = axis_vector.cross(up_vector);
      right_vector.normalized();
      tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
      q.normalize();
      pst.pose.orientation.x = q.getX();
      pst.pose.orientation.y = q.getY();
      pst.pose.orientation.z = q.getZ();
      pst.pose.orientation.w = q.getW();
    }
    else{
      // 二维路径段：Z 不变，只用 atan2(vy, vx) 计算 yaw，roll/pitch 为 0。
      double yaw = atan2(vy, vx);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pst.pose.orientation.x = q.getX();
      pst.pose.orientation.y = q.getY();
      pst.pose.orientation.z = q.getZ();
      pst.pose.orientation.w = q.getW();
    }

    // 对当前节点到下一节点做线性插值加密。这不会改变路径的几何形状，
    // 只是向折线段中增加采样点，便于局部规划器跟踪。
    geometry_msgs::msg::PoseStamped pst_inter_polate = pst;
    // 只有非末点后面才存在“当前节点 -> 下一节点”线段，才能在其中插值。
    // 末点没有后续线段，不做插值，只由 else 将末点本身加入 ros_path。
    if(it<path_id.size()-1){
      // 先保留原始图节点。
      ros_path.poses.push_back(pst);
      geometry_msgs::msg::PoseStamped last_pst = pst;
      // step 是路径段的比例，从 5% 增加到 95%；下一图节点由下一轮循环加入。
      for(double step=0.05;step<0.99;step+=0.05){

        // 在当前折线段上按比例插值 XYZ；插值点沿用当前节点的朝向。
        pst_inter_polate.pose.position.x = pst.pose.position.x + vx*step;
        pst_inter_polate.pose.position.y = pst.pose.position.y + vy*step;
        pst_inter_polate.pose.position.z = pst.pose.position.z + vz*step;
        double dx = pst_inter_polate.pose.position.x-last_pst.pose.position.x;
        double dy = pst_inter_polate.pose.position.y-last_pst.pose.position.y;
        double dz = pst_inter_polate.pose.position.z-last_pst.pose.position.z;
        // 只保留与上一个已输出点距离大于 0.1 m 的插值点，避免路径过密。
        if(sqrt(dx*dx+dy*dy+dz*dz)>0.1){
          ros_path.poses.push_back(pst_inter_polate);
          last_pst = pst_inter_polate;
        }
        
      }
    }
    else{
      // 最后一个图节点没有后续线段，直接追加到路径末尾。
      ros_path.poses.push_back(pst);
    }
    
  }
}

// 将起终点坐标匹配为 pcl_ground_ 中的节点索引，供后续 A* 搜索使用。
// 输入应已处于地面点云的坐标系，本函数不做 TF 转换，也不使用位姿朝向。
// 返回 true 仅表示两端匹配成功，不保证连通或无碰撞；false 时不要使用输出索引。
bool GlobalPlanner::getStartGoalID(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                                    unsigned int& start_id, unsigned int& goal_id){

  // 先匹配目标节点：搜索结果分别保存地面点索引和到查询点的平方距离。
  std::vector<int> pointIdxRadiusSearch_goal;
  std::vector<float> pointRadiusSquaredDistance_goal;
  pcl::PointXYZI pcl_goal;
  pcl_goal.x = goal.pose.position.x;
  pcl_goal.y = goal.pose.position.y;
  pcl_goal.z = goal.pose.position.z;

  // 在目标 XYZ 周围搜索半径 0.5 m 的三维球形邻域，高度差也计入距离。
  // 此处只选择节点；节点到准确目标的连接段由 makeROSPlan 后续校验。
  
  if(kdtree_ground_->radiusSearch (pcl_goal, 0.5, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal)<1){
    RCLCPP_WARN(this->get_logger(), "Goal is not found.");
    RCLCPP_WARN(this->get_logger(), "Using vertical search to find a goal on the ground.");
    bool second_search = false;
    // 首次未找到时，仅在地面图就绪后尝试向下搜索：XY 不变，
    // Z 每次降低 0.1 m，直到不再大于 -10 m；每层搜索半径为 0.3 m。
    if(graph_ready_){
      for(double z=goal.pose.position.z; z>-10;z-=0.1){
        pointIdxRadiusSearch_goal.clear();
        pointRadiusSquaredDistance_goal.clear();
        pcl_goal.z = z;
        if(kdtree_ground_->radiusSearch(pcl_goal, 0.3, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal,0)>0)
        {
          second_search = true;
          break;
        }
      }
      if(!second_search)
        return false;
    }
    else{
      return false;
    }
    // 注意现有行为：即使向下搜索找到节点，这里仍无条件返回失败。
    // 因此目前只有首次 0.5 m 搜索成功，才会继续选择 goal_id。
    // return false;
  }
  
  // 详细日志每次输出；普通模式每 5 秒输出一次节点匹配信息。
  if(enable_detail_log_){
    RCLCPP_WARN(this->get_logger(), "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, pointIdxRadiusSearch_goal[0], 
      pcl_ground_->points[pointIdxRadiusSearch_goal[0]].x, pcl_ground_->points[pointIdxRadiusSearch_goal[0]].y, pcl_ground_->points[pointIdxRadiusSearch_goal[0]].z);
  }
  else{
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, pointIdxRadiusSearch_goal[0], 
      pcl_ground_->points[pointIdxRadiusSearch_goal[0]].x, pcl_ground_->points[pointIdxRadiusSearch_goal[0]].y, pcl_ground_->points[pointIdxRadiusSearch_goal[0]].z);
  }
  
  // 取搜索结果首项作为目标节点，而不是把请求坐标直接加入地面图。
  goal_id = pointIdxRadiusSearch_goal[0];
  
  //--------------------------------------------------------------------------------------
  // 再匹配起点节点：机器人位姿通常对应机体原点，需要考虑其离地高度。
  std::vector<int> pointIdxRadiusSearch_start;
  std::vector<float> pointRadiusSquaredDistance_start;
  pcl::PointXYZI pcl_start;
  pcl_start.x = start.pose.position.x;
  pcl_start.y = start.pose.position.y;
  pcl_start.z = start.pose.position.z;

  if (start_ground_offset_ > 0.0) {
    // 将机体原点向下投影到预计地面高度，同时保留输入 start 不变。
    // 单独限制高度差，避免仅因 XY 更近就选中其他高度层的地面。
    pcl_start.z -= start_ground_offset_;
    // 先用覆盖水平/高度容差范围的球形邻域收集候选，随后分别筛选 XY 和 Z。
    kdtree_ground_->radiusSearch(pcl_start,
        std::hypot(find_start_tolerance_, start_ground_z_tolerance_),
        pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start);
    int best_id = -1;
    double best_xy = find_start_tolerance_ * find_start_tolerance_;  // 水平平方距离上限。
    double best_dz = start_ground_z_tolerance_;
    for (const int id : pointIdxRadiusSearch_start) {
      const auto& point = pcl_ground_->points[id];
      const double dx = point.x - pcl_start.x;
      const double dy = point.y - pcl_start.y;
      const double xy = dx * dx + dy * dy;
      const double dz = std::abs(point.z - pcl_start.z);
      // 高度差必须合格；优先选择水平距离最小的节点，水平距离相同时选高度差较小者。
      if (dz <= start_ground_z_tolerance_ &&
          (xy < best_xy || (xy == best_xy && dz <= best_dz))) {
        best_id = id;
        best_xy = xy;
        best_dz = dz;
      }
    }
    // 只保留筛选出的最佳节点；没有合格候选时维持空结果。
    pointIdxRadiusSearch_start.clear();
    if (best_id >= 0) pointIdxRadiusSearch_start.push_back(best_id);
  } else {
    // 未启用高度投影时，直接围绕机体 XYZ 做三维半径搜索。
    kdtree_ground_->radiusSearch(pcl_start, find_start_tolerance_,
        pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start);
  }
  // 起点无匹配时返回失败，并记录机体高度、查询高度和容差，便于排查投影问题。
  if (pointIdxRadiusSearch_start.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000,
        "Start ground not found: body=(%.3f, %.3f, %.3f), query_z=%.3f, "
        "search_tolerance=%.3f, ground_offset=%.3f, z_tolerance=%.3f",
        start.pose.position.x, start.pose.position.y, start.pose.position.z,
        pcl_start.z, find_start_tolerance_, start_ground_offset_, start_ground_z_tolerance_);
    return false;
  }
  // 输出实际选中地面的水平偏移和机体离地高差，用于核对高度偏移配置。
  const auto& selected_ground = pcl_ground_->points[pointIdxRadiusSearch_start[0]];
  RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000,
      "Start ground projection: horizontal_distance=%.3f, body_height=%.3f, ground_z=%.3f",
      std::hypot(selected_ground.x - pcl_start.x, selected_ground.y - pcl_start.y),
      start.pose.position.z - selected_ground.z, selected_ground.z);
  
  if(enable_detail_log_){
    RCLCPP_WARN(this->get_logger(), "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, pointIdxRadiusSearch_start[0], 
      pcl_ground_->points[pointIdxRadiusSearch_start[0]].x, pcl_ground_->points[pointIdxRadiusSearch_start[0]].y, pcl_ground_->points[pointIdxRadiusSearch_start[0]].z);
  }
  else{
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, pointIdxRadiusSearch_start[0], 
      pcl_ground_->points[pointIdxRadiusSearch_start[0]].x, pcl_ground_->points[pointIdxRadiusSearch_start[0]].y, pcl_ground_->points[pointIdxRadiusSearch_start[0]].z);

  }
  // 两端索引均已确定，调用方可继续执行 A* 及连接段碰撞检查。
  start_id = pointIdxRadiusSearch_start[0];

  return true;

}

// 处理一次 /get_plan Action 请求：获取起终点、计算路径并返回结果。
// 此处只负责全局规划，不发送运动目标或速度指令；WebUI 路径预览也调用此接口。
void GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  
  // 请求包含目标位姿 goal 和规划开关 activate_threading；起点由当前机器人位姿确定。
  const auto goal = goal_handle->get_goal();

  // 静态感知层尚未准备好时无法可靠规划，直接以 ABORTED 状态返回空结果。
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Received the request before static layer is ready");
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->abort(result);
    return;
  }

  // false 表示本次不执行规划，正常结束请求并返回空结果。
  // true 在本函数中只触发一次计算，不会启动持续重规划循环。
  if(!goal_handle->get_goal()->activate_threading){
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Deactivate thread");
    goal_handle->succeed(result);
    return;
  }

  // 从感知模块获取全局坐标系下的机器人位姿，作为本次规划的起点。
  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *clock_, 5000,
    "Planning request: start=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f), "
    "static_ground_nodes=%zu, graph_ready=%d",
    start.pose.position.x, start.pose.position.y, start.pose.position.z,
    goal->goal.pose.position.x, goal->goal.pose.position.y,
    goal->goal.pose.position.z, pcl_ground_->size(), graph_ready_);

  // 实际找路交给 makeROSPlan：起终点匹配地面节点 → A* 搜索 → 路径转换，
  // 并校验起点连接段、目标连接段及目标朝向下的车体碰撞；失败时返回空路径。
  auto ros_path = makeROSPlan(start, goal->goal);

  // 空路径统一作为规划失败处理；具体失败环节由 makeROSPlan 内部日志说明。
  // 将空路径写回 Action 结果，避免把上一轮路径作为本次结果返回。
  if(ros_path.poses.empty()){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 5000,
      "Planning result: failed, no path points; start=(%.2f, %.2f, %.2f), "
      "goal=(%.2f, %.2f, %.2f), graph_ready=%d",
      start.pose.position.x, start.pose.position.y, start.pose.position.z,
      goal->goal.pose.position.x, goal->goal.pose.position.y,
      goal->goal.pose.position.z, graph_ready_);
    global_plan_result_->path = ros_path;
    goal_handle->abort(global_plan_result_);
  }
  else{
    //postSmoothPath(path, smoothed_path);
    // 非空路径同时通过话题发布和 Action 结果返回：话题供订阅者使用，
    // succeed 将本次请求标记为 SUCCEEDED；是否执行运动由导航任务层决定。
    pub_path_->publish(ros_path);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *clock_, 5000,
      "Planning result: success, path_points=%zu, start=(%.2f, %.2f, %.2f), "
      "goal=(%.2f, %.2f, %.2f)",
      ros_path.poses.size(), start.pose.position.x, start.pose.position.y,
      start.pose.position.z, goal->goal.pose.position.x,
      goal->goal.pose.position.y, goal->goal.pose.position.z);
    global_plan_result_->path = ros_path;
    goal_handle->succeed(global_plan_result_);
  }
  
}

// 将全局坐标系下的起终点转换为可执行的几何路径：搜索地面节点路径，
// 再校验它与实际起终点之间的连接。任一阶段失败都返回 poses 为空的 Path。
nav_msgs::msg::Path GlobalPlanner::makeROSPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal){
  
  // 规划期间锁住地面数据，防止节点索引和 KD-tree 在地图更新时失去对应关系。
  std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
  unsigned int start_id = 0;
  unsigned int goal_id = 0;
  std::vector<unsigned int> path;  // A* 输出的地面点索引序列，尚未转换为 ROS 位姿。
  // 预留的平滑结果容器；本函数当前没有执行路径平滑。
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  nav_msgs::msg::Path ros_path;

  // 将连续坐标匹配到离散地面节点；找不到符合条件的起点或终点则无法搜索。
  if(!getStartGoalID(start, goal, start_id, goal_id)){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 5000,
      "Cannot resolve start/goal to ground graph: start=(%.2f, %.2f, %.2f), "
      "goal=(%.2f, %.2f, %.2f), start_tolerance=%.2f, graph_ready=%d",
      start.pose.position.x, start.pose.position.y, start.pose.position.z,
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      find_start_tolerance_, graph_ready_);
    return ros_path;
  }

  // 按配置选择在点云邻域上搜索，或使用预先建立连边的地面图搜索。
  if(!use_pre_graph_)
    a_star_planner_->getPath(start_id, goal_id, path);
  else
    a_star_planner_pre_graph_->getPath(start_id, goal_id, path);

  // getStartGoalID() 成功只表示起点和终点附近都找到了地面节点。
  // getPath() 返回非空 path 后，才表示 start_id 到 goal_id 在当前地面图上可达。
  if(path.empty()){
    if(enable_detail_log_)
      RCLCPP_WARN(this->get_logger(), "No path found from node %u to %u; graph nodes=%zu", start_id, goal_id, pcl_ground_->size());
    else
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "No path found from node %u to %u; graph nodes=%zu", start_id, goal_id, pcl_ground_->size());
    return ros_path;
  }
  else{
    if(enable_detail_log_)
      RCLCPP_INFO(this->get_logger(), "Path found from node %u to %u: graph path nodes=%zu", start_id, goal_id, path.size());
    else
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "Path found from node %u to %u: graph path nodes=%zu", start_id, goal_id, path.size());
    // 将节点索引序列转换为带坐标和朝向的 nav_msgs::msg::Path。
    getROSPath(path, ros_path);
    // 启用机体到地面的高度偏移时，补齐机器人当前 XY 到首个地面节点的连接段。
    // 连接段使用首节点的地面 Z，避免把机体原点高度当成路径的地面高度。
    if (start_ground_offset_ > 0.0 && !ros_path.poses.empty()) {
      const auto anchor = ros_path.poses.front();
      const double dx = anchor.pose.position.x - start.pose.position.x;
      const double dy = anchor.pose.position.y - start.pose.position.y;
      const double length = std::hypot(dx, dy);
      // 直接用直线插值连接机身当前 XY 和 A* 首个地面节点。
      // 连接段不额外执行车体碰撞检查或连续地面支撑检查。
      if (length > 0.001) {
        // 以不超过 5 cm 的水平间距插值；不包含 anchor 本身，原路径已包含该点。
        std::vector<geometry_msgs::msg::PoseStamped> connector;
        const int steps = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
        connector.reserve(steps);
        for (int i = 0; i < steps; ++i) {
          auto pose = anchor;
          const double t = static_cast<double>(i) / steps;
          pose.pose.position.x = start.pose.position.x + dx * t;
          pose.pose.position.y = start.pose.position.y + dy * t;
          connector.push_back(pose);
        }
        ros_path.poses.insert(ros_path.poses.begin(), connector.begin(), connector.end());
        RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 5000,
            "Added straight start connector: horizontal_gap=%.3f m, samples=%zu, ground_z=%.3f",
            length, connector.size(), anchor.pose.position.z);
      }
    }
    // A* 终点是离散地面节点，可能与请求的准确目标有偏差。
    // 直接用直线插值连接图终点和准确目标，不额外执行碰撞或地面支撑检查。
    const auto graph_goal = ros_path.poses.back();
    const double connector_dx = goal.pose.position.x - graph_goal.pose.position.x;
    const double connector_dy = goal.pose.position.y - graph_goal.pose.position.y;
    const double connector_dz = goal.pose.position.z - graph_goal.pose.position.z;
    const double connector_length = std::sqrt(
      connector_dx * connector_dx + connector_dy * connector_dy +
      connector_dz * connector_dz);
    if (connector_length > 0.001) {
      // 图终点已在 ros_path 中，准确目标在下方单独追加；
      // 因此这里只生成两者之间的中间点，间距不超过 5 cm。
      const int steps = std::max(
        1, static_cast<int>(std::ceil(connector_length / 0.05)));
      for (int i = 1; i < steps; ++i) {
        auto pose = goal;
        const double t = static_cast<double>(i) / steps;
        pose.pose.position.x = graph_goal.pose.position.x + connector_dx * t;
        pose.pose.position.y = graph_goal.pose.position.y + connector_dy * t;
        pose.pose.position.z = graph_goal.pose.position.z + connector_dz * t;
        ros_path.poses.push_back(pose);
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 5000,
          "Added straight goal connector: length=%.3f m, intermediate_samples=%d",
          connector_length, std::max(0, steps - 1));
    }
    // 保留请求的准确目标位置和朝向，让下游控制器追踪到目标位姿。
    ros_path.poses.push_back(goal);
    return ros_path;
  }
}

void GlobalPlanner::getStaticGraphFromPerception3D(){
  
  //@Calculate node weight
  
  if(!has_initialized_){
    has_initialized_ = true;
    if(a_star_expanding_radius_ >= perception_3d_ros_->getGlobalUtils()->getInscribedRadius()*2){
      RCLCPP_WARN(this->get_logger(), "Expanding radius is much larger than InscribedRadius, the planning time will be increased.");
    }
    if(!use_pre_graph_){
      a_star_planner_ = std::make_shared<A_Star_on_Graph>(
        pcl_ground_, pcl_map_, perception_3d_ros_, a_star_expanding_radius_, footprint_);
      a_star_planner_->setupTurningWeight(turning_weight_);
    }
    else{
      a_star_planner_pre_graph_ = std::make_shared<A_Star_on_PreGraph>(
        pcl_ground_, pcl_map_, static_graph_, perception_3d_ros_, a_star_expanding_radius_, footprint_);
      a_star_planner_pre_graph_->setupTurningWeight(turning_weight_);
    }
  }
  else{
    a_star_planner_->updateGraph(pcl_ground_);
  }

  pubWeight();

  RCLCPP_INFO(this->get_logger(), "Publish weighted ground point cloud.");
  graph_ready_ = true;
}


void GlobalPlanner::pubWeight(){

  /*
  When generate_static_graph is true, the static_layer will generate sGraph_ptr_.
  And there will be wo condition:
  1. enable_edge_detection = false; This arg will push zero element to node_weight_ 
  */

  pcl::PointCloud<pcl::PointXYZI>::Ptr weighted_pc (new pcl::PointCloud<pcl::PointXYZI>);
  
  unsigned long node_weight_size = perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_->getNodeWeightSize();
  bool is_node_weight = true;
  if(node_weight_size<=0){
    node_weight_size = perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_->getSize();
    is_node_weight = false;
  }

  for(auto it=0; it<node_weight_size; it++){

    pcl::PointXYZI ipt;

    ipt.x = pcl_ground_->points[it].x;
    ipt.y = pcl_ground_->points[it].y;
    ipt.z = pcl_ground_->points[it].z;
    if(is_node_weight)
      ipt.intensity = static_graph_.getNodeWeight(it);
    else
      ipt.intensity = 0;
    weighted_pc->push_back(ipt);

  }
  weighted_pc->header.frame_id = global_frame_;
  sensor_msgs::msg::PointCloud2 ros_msg_weighted_pc;
  ros_msg_weighted_pc.header.stamp = clock_->now();
  pcl::toROSMsg(*weighted_pc, ros_msg_weighted_pc);
  pub_weighted_pc_->publish(ros_msg_weighted_pc);
}

void GlobalPlanner::pubStaticGraph(){
 
  //@edge visualization 
  //@This is just for visulization, therefore reduce edges to let rviz less lag
  
  std::set<std::pair<unsigned int, unsigned int>> duplicate_check;
  visualization_msgs::msg::MarkerArray markerArray;
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = global_frame_;
  markerEdge.header.stamp = clock_->now();
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.pose.orientation.w = 1.0;
  markerEdge.ns = "edges";
  markerEdge.id = 3;
  markerEdge.scale.x = 0.03;
  markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
  markerEdge.color.a = 0.2;

  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();

  int cnt = 0;
  for(auto it = (*static_graph).begin();it!=(*static_graph).end();it++){
    geometry_msgs::msg::Point p;
    p.x = pcl_ground_->points[(*it).first].x;
    p.y = pcl_ground_->points[(*it).first].y;
    p.z = pcl_ground_->points[(*it).first].z;
    for(auto it_set = (*it).second.begin();it_set != (*it).second.end();it_set++){

      std::pair<unsigned int, unsigned int> edge_marker, edge_marker_inverse;
      edge_marker.first = (*it).first;
      edge_marker.second = (*it_set).first;
      edge_marker_inverse.first = (*it_set).first;
      edge_marker_inverse.second = (*it).first;
      if( !duplicate_check.insert(edge_marker).second )
      {   
        continue;
      }
      if( !duplicate_check.insert(edge_marker_inverse).second )
      {   
        continue;
      }
      markerEdge.points.push_back(p);
      p.x = pcl_ground_->points[(*it_set).first].x;
      p.y = pcl_ground_->points[(*it_set).first].y;
      p.z = pcl_ground_->points[(*it_set).first].z;     
      markerEdge.points.push_back(p);
      markerEdge.id = cnt;
      cnt++;
    }
  }
  markerArray.markers.push_back(markerEdge);
  pub_static_graph_->publish(markerArray);
}

bool GlobalPlanner::isFootprintPoseClear(
  const pcl::PointXYZI & center, double yaw) const
{
  if (use_pre_graph_) {
    return a_star_planner_pre_graph_ &&
      a_star_planner_pre_graph_->isFootprintPoseClear(center, yaw);
  }
  return a_star_planner_ && a_star_planner_->isFootprintPoseClear(center, yaw);
}

bool GlobalPlanner::isFootprintSweepClear(
  const pcl::PointXYZI & start, const pcl::PointXYZI & end) const
{
  if (use_pre_graph_) {
    return a_star_planner_pre_graph_ &&
      a_star_planner_pre_graph_->isFootprintSweepClear(start, end);
  }
  return a_star_planner_ && a_star_planner_->isFootprintSweepClear(start, end);
}

bool GlobalPlanner::isFootprintSweepClearAtYaw(
  const pcl::PointXYZI & start, const pcl::PointXYZI & end,
  double yaw) const
{
  if (use_pre_graph_) {
    return a_star_planner_pre_graph_ &&
      a_star_planner_pre_graph_->isFootprintSweepClearAtYaw(start, end, yaw);
  }
  return a_star_planner_ &&
    a_star_planner_->isFootprintSweepClearAtYaw(start, end, yaw);
}

}
