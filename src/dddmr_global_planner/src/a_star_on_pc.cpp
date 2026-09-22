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
#include <global_planner/a_star_on_pc.h>

AstarList::AstarList(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up){
  pc_original_z_up_ = pc_original_z_up;
  kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_ground_->setInputCloud(pc_original_z_up_);
}

void AstarList::Initial(){
  as_list_.clear(); 
  for(unsigned int it=0; it!=pc_original_z_up_->points.size();it++){
    Node_t new_node = {.self_index=0, .g=0, .h=0, .f=0, .parent_index=0, .is_closed=false, .is_opened=false};
    as_list_[it] = new_node;
  }
  f_priority_set_.clear();
}

Node_t AstarList::getNode(unsigned int node_index){

  return as_list_[node_index];
}

float AstarList::getGVal(Node_t& a_node){
  return as_list_[a_node.self_index].g;
}

void AstarList::closeNode(Node_t& a_node){
  as_list_[a_node.self_index].is_closed = true;
}

void AstarList::updateNode(Node_t& a_node){
  as_list_[a_node.self_index] = a_node;
  f_p_ afp;
  afp.first = a_node.f; //made minimum f to be top so we can pop it
  afp.second = a_node.self_index;
  f_priority_set_.insert(afp);
  //ROS_DEBUG("Add node ---> %u with g: %f, h: %f, f: %f",a_node.self_index, a_node.g, a_node.h, a_node.f);
}

Node_t AstarList::getNode_wi_MinimumF(){
  auto first_it = f_priority_set_.begin();
  Node_t m_node = as_list_[(*first_it).second];
  if(!m_node.is_closed){
    f_priority_set_.erase(first_it);
    return m_node;
  }
  
  //Because we updateNode node even when new g value is smaller than that in openlist
  //We will have duplicate f value in the f_priority_set_
  int concern_cnt = 0;
  while(m_node.is_closed && !f_priority_set_.empty()){
    concern_cnt++;
    f_priority_set_.erase(first_it);
    first_it = f_priority_set_.begin();
    m_node = as_list_[(*first_it).second];
  }
  return m_node;
}

bool AstarList::isClosed(unsigned int node_index){
  return as_list_[node_index].is_closed;
}

bool AstarList::isOpened(unsigned int node_index){
  return as_list_[node_index].is_opened;
}

bool AstarList::isFrontierEmpty(){
  return f_priority_set_.empty();
}

//@----------------------------------------------------------------------------------------

A_Star_on_Graph::A_Star_on_Graph(
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map,
                                  std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
                                  double a_star_expanding_radius,
                                  const CuboidFootprint & footprint){
  
  perception_ros_ = perception_ros;
  pc_original_z_up_ = pc_original_z_up;
  pc_map_ = pc_map;
  a_star_expanding_radius_ = a_star_expanding_radius;
  footprint_ = footprint;
  kdtree_map_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  if (pc_map_ && !pc_map_->empty()) {
    kdtree_map_->setInputCloud(pc_map_);
  }
  ASLS_ = new AstarList(pc_original_z_up_);
}

A_Star_on_Graph::~A_Star_on_Graph(){
  if(ASLS_)
    delete ASLS_;
}

void A_Star_on_Graph::updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up){
  ASLS_->pc_original_z_up_ = pc_original_z_up;
  ASLS_->kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  ASLS_->kdtree_ground_->setInputCloud(pc_original_z_up_);
}

double A_Star_on_Graph::getPitchFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1, s1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  s1 = sqrt(vx1*vx1 + vy1*vy1);
  //@ calculate vector: current -> expanding
  float vx2, vy2, s2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  s2 = sqrt(vx2*vx2 + vy2*vy2);

  float pitch = fabs(m_pcl_current_parent.z - m_pcl_expanding.z)/(s1+s2);

  return pitch;
}

double A_Star_on_Graph::getThetaFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  //@ calculate vector: current -> expanding
  float vx2, vy2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  float cos_theta = (vx1*vx2 + vy1*vy2)/(sqrt(vx1*vx1+vy1*vy1)*sqrt(vx2*vx2+vy2*vy2));
  if(fabs(cos_theta)>1)
    cos_theta = 1.0;
  double theta_of_vector = acos(cos_theta);
  if(vx1==0 && vy1==0)
    theta_of_vector = 0;
  else if(vx2==0 && vy2==0)
    theta_of_vector = 0;
  else if(fabs(fabs(vx1)-fabs(vx2))<=0.0001)
    theta_of_vector = 0;
  
  if(fabs(theta_of_vector)<=0.345)//cap
    theta_of_vector = 0.0;

  return theta_of_vector;
}

bool A_Star_on_Graph::isFootprintSweepClear(
  const pcl::PointXYZI & pcl_current,
  const pcl::PointXYZI & pcl_expanding) const
{
  // 检查车体长方体从当前地面节点移动到候选节点时是否全程无碰撞。
  // cuboidFootprintSweepClear() 会沿候选边按 footprint_.sample_step 采样，
  // 在每个采样位置放置车体包围盒；任意一处碰撞即返回 false。
  //
  // 三类障碍数据使用 && 串联，按短路规则依次检查。任意一类不安全，
  // 整条候选边就会被 getPath() 放弃。
  return cuboidFootprintSweepClear(
      // 静态地图点保留真实高度，check_height=true 检查完整三维车体。
      pcl_current, pcl_expanding, footprint_, kdtree_map_,
      pc_map_ ? pc_map_->size() : 0, true) &&
    cuboidFootprintSweepClear(
      // 实时三维观测与局部规划器使用同一数据；其中可能含有尚未投影到
      // lethal 地面图的孤立障碍点，因此同样启用高度检查。
      pcl_current, pcl_expanding, footprint_, kdtree_observation_,
      pc_observation_ ? pc_observation_->size() : 0, true) &&
    cuboidFootprintSweepClear(
      // lethal 点已投影到地面图，check_height=false 只检查 XY 占地范围。
      pcl_current, pcl_expanding, footprint_, kdtree_lethal_,
      pc_lethal_ ? pc_lethal_->size() : 0, false);
}

bool A_Star_on_Graph::isFootprintPoseClear(
  const pcl::PointXYZI & center, double yaw) const
{
  return cuboidFootprintPoseClear(
      center, yaw, footprint_, kdtree_map_, pc_map_ ? pc_map_->size() : 0, true) &&
    cuboidFootprintPoseClear(
      center, yaw, footprint_, kdtree_observation_,
      pc_observation_ ? pc_observation_->size() : 0, true) &&
    cuboidFootprintPoseClear(
      center, yaw, footprint_, kdtree_lethal_,
      pc_lethal_ ? pc_lethal_->size() : 0, false);
}

bool A_Star_on_Graph::isFootprintSweepClearAtYaw(
  const pcl::PointXYZI & pcl_current,
  const pcl::PointXYZI & pcl_expanding, double yaw) const
{
  return cuboidFootprintSweepClearAtYaw(
      pcl_current, pcl_expanding, yaw, footprint_, kdtree_map_,
      pc_map_ ? pc_map_->size() : 0, true) &&
    cuboidFootprintSweepClearAtYaw(
      pcl_current, pcl_expanding, yaw, footprint_, kdtree_observation_,
      pc_observation_ ? pc_observation_->size() : 0, true) &&
    cuboidFootprintSweepClearAtYaw(
      pcl_current, pcl_expanding, yaw, footprint_, kdtree_lethal_,
      pc_lethal_ ? pc_lethal_->size() : 0, false);
}

// 在地面点云上执行 A*：通过半径搜索动态寻找邻接节点，无需预先建立连边。
// start/goal 是地面点索引；成功后 path 保存从起点到终点的索引序列。
// 调用方应传入空 path：本函数不主动清空它，搜索失败时也不会写入新路径。
void A_Star_on_Graph::getPath(
  unsigned int start, unsigned int goal,
  std::vector<unsigned int>& path){

  //RCLCPP_INFO(rclcpp::get_logger("astar"),"Start: %u, Goal: %u", start, goal);

  // 初始化起点：累计代价 g 为 0，父节点指向自身，作为后续回溯的终止标记。
  // 初始 f 使用起终点三维直线距离；此处 h 字段仍为 0，后续候选节点按 f=g+h 计算。
  // pc_original_z_up_是地面点云指针
  pcl::PointXYZI pcl_goal = pc_original_z_up_->points[goal];
  pcl::PointXYZI pcl_start = pc_original_z_up_->points[start];
  float f = sqrt(pcl::geometry::squaredDistance(pcl_start, pcl_goal));
  Node_t current_node = {.self_index=start, .g=0, .h=0, .f=f, .parent_index=start, .is_closed=false, .is_opened=true};

  // 重置本轮搜索状态，并将起点加入待扩展集合（open/frontier）。
  ASLS_->Initial();
  ASLS_->updateNode(current_node);
  
  // 获取软膨胀代价参数：参考半径与指数衰减速率。
  // max_obstacle_distance 当前仅被读取，未参与本函数后续计算。
  double inscribed_radius = perception_ros_->getGlobalUtils()->getInscribedRadius();
  double inflation_descending_rate = perception_ros_->getGlobalUtils()->getInflationDescendingRate();
  double max_obstacle_distance = perception_ros_->getGlobalUtils()->getMaxObstacleDistance();
  
  // 汇总各感知层的致命障碍点与观测点，为本轮车体碰撞检查准备数据。
  auto stacked_perception = perception_ros_->getStackedPerception();
  stacked_perception->aggregateLethal();
  stacked_perception->aggregateObservations();
  // 深拷贝这两类点云，整轮搜索使用固定快照，避免传感器更新替换点云影响检查。
  // 下方 dGraph 和节点权重仍从感知模块读取，这里并未快照所有感知数据。
  pc_lethal_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
    *perception_ros_->getSharedDataPtr()->aggregate_lethal_);
  pc_observation_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
    *perception_ros_->getSharedDataPtr()->aggregate_observation_);
  // 分别建立障碍搜索树；空点云不设置输入，碰撞辅助函数按点数处理空数据。
  kdtree_lethal_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_observation_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  if(!pc_lethal_->empty())
    kdtree_lethal_->setInputCloud(pc_lethal_);
  if(!pc_observation_->empty())
    kdtree_observation_->setInputCloud(pc_observation_);
    

  while(!ASLS_->isFrontierEmpty()){ 
    // 从按 f 排序的待扩展集合中取出最小代价节点，无需每轮遍历整个集合。
    current_node = ASLS_->getNode_wi_MinimumF();

    //RCLCPP_INFO(rclcpp::get_logger("astar"), "Expand node: %u", current_node.self_index);
    // 搜索当前地面点附近的候选邻居；扩展半径决定本轮可尝试连接的节点范围。
    pcl::PointXYZI pcl_now = pc_original_z_up_->points[current_node.self_index];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    ASLS_->kdtree_ground_->radiusSearch(pcl_now, a_star_expanding_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    // 使用邻域平均 intensity 作为地面边缘代价，降低高代价区域中孤立低代价点的吸引力。
    // 此处假定半径搜索结果非空，代码没有单独处理邻居数量为 0 的除法。
    float avg_intensity = 0.0;
    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      avg_intensity += pc_original_z_up_->points[pointIdxRadiusSearch[it]].intensity;
    }
    avg_intensity = avg_intensity/pointIdxRadiusSearch.size();

    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      
      int current_expanding_index = pointIdxRadiusSearch[it];
      // 搜索返回平方距离，开方得到当前节点到候选节点的三维边长。
      float current_expanding_g = sqrt(pointRadiusSquaredDistance[it]);

      // 查询候选节点的 dGraph 障碍距离，用于软膨胀惩罚。
      double dGraphValue = perception_ros_->get_min_dGraphValue(current_expanding_index);

      pcl::PointXYZI pcl_current = pc_original_z_up_->points[current_node.self_index];
      pcl::PointXYZI pcl_current_parent = pc_original_z_up_->points[current_node.parent_index];
      pcl::PointXYZI pcl_expanding = pc_original_z_up_->points[current_expanding_index];

      // 沿候选边扫掠车体长方体，检测静态地图、观测点和致命障碍；碰撞则跳过该边。
      // 硬碰撞由长方体检查决定，下面的圆形距离只参与软膨胀代价。
      if(!isFootprintSweepClear(pcl_current, pcl_expanding))
        continue;
      
      // 正衰减率下，离障碍越近惩罚越大；距离等于参考半径时代价为 1。
      double factor = exp(-1.0 * inflation_descending_rate * (dGraphValue - inscribed_radius));

      // 根据“父节点 → 当前节点 → 候选节点”的方向变化计算转弯惩罚。
      double theta = getThetaFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding);
      
      //if(getPitchFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding)>0.2)
      //  continue;
      
      // g 累加：已有路径代价 + 边长 + 膨胀代价 + 静态图节点权重
      //         + 转角×转弯权重 + 当前邻域的地面边缘代价。
      float ground_edge_weight = avg_intensity;
      float node_weight = perception_ros_->getSharedDataPtr()->sGraph_ptr_->getNodeWeight(current_expanding_index);
      float new_g = current_node.g + current_expanding_g + factor * 1.0 + node_weight + theta*turning_weight_ + ground_edge_weight;
      // h 用候选点到目标的三维直线距离估计剩余路程，f=g+h 用于扩展排序。
      float new_h = sqrt(pcl::geometry::squaredDistance(pcl_expanding, pcl_goal));
      float new_f = new_g + new_h;

      // 记录从哪个节点到达候选点，找到目标后沿 parent_index 回溯路径。
      Node_t new_node = {
        .self_index=static_cast<unsigned int>(current_expanding_index),
        .g=new_g, .h=new_h, .f=new_f,
        .parent_index=current_node.self_index,
        .is_closed=false, .is_opened=true};

      // 已完成扩展的节点直接跳过；本实现不重新打开 closed 节点。
      if(ASLS_->isClosed(current_expanding_index))
        continue;
      // 已在 open 中时，仅当新路径的累计代价更小，才更新代价和父节点。
      else if(ASLS_->isOpened(current_expanding_index)){
        if(ASLS_->getGVal(new_node)>new_g){
          ASLS_->updateNode(new_node);          
        }
      }
      // 首次发现的节点加入待扩展集合。
      else{
        ASLS_->updateNode(new_node);
      }
        
      
    }

    // 当前节点的所有候选边处理完毕，将其标记为 closed。
    ASLS_->closeNode(current_node);

    // 目标进入 closed 后完成搜索，从目标沿父节点回溯到起点。
    if(ASLS_->isClosed(goal)){
      Node_t trace_back = ASLS_->getNode(goal);
      while(trace_back.self_index!=trace_back.parent_index){
        path.push_back(trace_back.self_index);
        trace_back = ASLS_->getNode(trace_back.parent_index);
      }
      path.push_back(trace_back.self_index);// 补入父节点指向自身的起点。
      std::reverse(path.begin(),path.end()); // 回溯顺序为终点到起点，反转后得到正向路径。
      break;
    }

    // 若目标尚未关闭则继续扩展；frontier 耗尽仍未到达目标时结束，不生成路径。
  }

}
