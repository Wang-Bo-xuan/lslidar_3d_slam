#include "map_builder.h"
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace lidar_slam_3d
{

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

MapBuilder::MapBuilder() :
  first_point_cloud_(true), sequence_num_(0),
  pose_(Eigen::Matrix4f::Identity()), last_update_pose_(Eigen::Matrix4f::Identity()),
  submap_size_(30), voxel_grid_leaf_size_(1.0), map_update_distance_(2.0), enable_optimize_(false),
  loop_search_distance_(25.0), loop_min_chain_size_(5), loop_min_fitness_score_(1.5),
  loop_keyframe_skip_(20), loop_constraint_count_(0), optimize_every_n_constraint_(10)
{
  //获取参数服务器参数
  ros::NodeHandle private_nh("~");
  private_nh.param("voxel_grid_leaf_size_", voxel_grid_leaf_size_, 1.0);
  private_nh.param("map_update_distance_", map_update_distance_, 2.0);
  private_nh.param("enable_loop_closeure", enable_optimize_, false);

  //初始化ndt配准器
  ndt_.setTransformationEpsilon(0.01);
  ndt_.setStepSize(0.1);
  ndt_.setResolution(1.0);
  ndt_.setMaximumIterations(30);

  //初始化优化器
  SlamBlockSolver::LinearSolverType* linear_solver = new SlamLinearSolver;
  SlamBlockSolver* solver_ptr = new SlamBlockSolver(linear_solver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
  optimizer_.setAlgorithm(solver);
  optimizer_.setVerbose(false);
}

void MapBuilder::downSample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sampled_cloud)
{//使用pcl自带的体素滤波，voxel_size在初始化时确认
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
  voxel_grid_filter.setInputCloud(input_cloud);
  voxel_grid_filter.filter(*sampled_cloud);
}

void MapBuilder::addVertex(const KeyFrame::Ptr& key_frame)
{//向位姿因子图中添加一个顶点
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(key_frame->getId());
  vertex->setEstimate(Eigen::Isometry3d(key_frame->getPose().cast<double>()));
  optimizer_.addVertex(vertex);
}

void MapBuilder::addEdge(const KeyFrame::Ptr& source, const Eigen::Matrix4f& source_pose,
                         const KeyFrame::Ptr& target, const Eigen::Matrix4f& target_pose,
                         const Eigen::Matrix<double, 6, 6>& information)
{//为位姿因子图添加边/约束
  static int edge_count = 0;

  //初始化姿态李群
  g2o::EdgeSE3* edge = new g2o::EdgeSE3;

  //分别获取源及目标的顶点信息
  int source_id = source->getId();
  int target_id = target->getId();

  edge->vertices()[0] = optimizer_.vertex(source_id);
  edge->vertices()[1] = optimizer_.vertex(target_id);

  //计算出相对位姿
  Eigen::Isometry3d relative_pose((source_pose.inverse() * target_pose).cast<double>());
  edge->setId(edge_count);
  edge->setMeasurement(relative_pose);
  edge->setInformation(information);
  edge_count++;

  //添加边
  optimizer_.addEdge(edge);
}

KeyFrame::Ptr MapBuilder::getClosestKeyFrame(const KeyFrame::Ptr& key_frame,
                                             const std::vector<KeyFrame::Ptr>& candidates)
{
  //获取关键帧的位置
  Eigen::Vector3f pt1 = key_frame->getPose().block<3, 1>(0, 3);
  float min_distance = std::numeric_limits<float>::max();
  int id;

  //遍历约束链中所有的关键帧
  for(const KeyFrame::Ptr& frame : candidates)
  {
    //获取关键帧位置并计算与当前帧位置差
    Eigen::Vector3f pt2 = frame->getPose().block<3, 1>(0, 3);
    float distance = (pt1 - pt2).norm();

    //保存距离最小的关键帧的id
    if(distance < min_distance)
    {
      min_distance = distance;
      id = frame->getId();
    }
  }

  //返回结果
  return key_frames_[id];
}

void MapBuilder::detectLoopClosure(const KeyFrame::Ptr& key_frame)
{//闭环检测
  std::vector<KeyFrame::Ptr> cloud_chain;
  std::vector<std::vector<KeyFrame::Ptr>> cloud_chains;

  //获取关键帧总数及当前帧位置
  int n = key_frames_.size();
  Eigen::Vector3f pt1 = key_frame->getPose().block<3, 1>(0, 3);

  //遍历所有关键帧
  for(int i = 0; i < n; ++i)
  {
    //获取每个关键帧的位置并计算与当前帧的位置差
    Eigen::Vector3f pt2 = key_frames_[i]->getPose().block<3, 1>(0, 3);
    float distance = (pt1 - pt2).norm();

    //如果位置差小于预设阈值，可能是回环
    if(distance < loop_search_distance_)
    {
      //如果不是刚刚经过的关键帧
      if(key_frames_[i]->getId() < key_frame->getId() - loop_keyframe_skip_)
      {
        //保存到同一个约束链中
        cloud_chain.push_back(key_frames_[i]);
      }
      else
      {
        //相邻关键帧位置差满足阈值要求不是真的回环，清空约束链
        cloud_chain.clear();
      }
    }
    else
    {//如果位置差不满足预设阈值，则回环已经结束(约束链不为空)或不是回环(约束链为空)
      //如果约束链中节点数量大于预设阈值，一定有回环
      if(cloud_chain.size() > loop_min_chain_size_)
      {
        //保存上一个回环约束链
        cloud_chains.push_back(cloud_chain);
        std::cout << "\033[36m" << "Find loop candidates. Keyframe chain size "
                  << cloud_chain.size() << std::endl;

        //清空约束链
        cloud_chain.clear();
      }
      else
      {//约束链中节点数量小于预设阈值，则场景相似但无回环或没有约束节点
        //清空约束链
        cloud_chain.clear();
      }
    }
  }

  //如果约束链数组中没有一个条约束链存在，则无回环
  if(cloud_chains.empty())
  {
    //本次回环检测结束
    return;
  }

  //如果有闭环
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  //将当前帧点云下采样，加速配准
  downSample(key_frame->getCloud(), sampled_cloud);

  //遍历所有约束链数组
  for(const std::vector<KeyFrame::Ptr>& chain : cloud_chains)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    //把同一个约束链中的点云对应好位姿累加起来，作为配准目标点云
    for(const KeyFrame::Ptr& frame : chain)
    {
      pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
      pcl::transformPointCloud(*(frame->getCloud()), transformed_cloud, frame->getPose());
      *target_cloud += transformed_cloud;
    }

    //使用当前点云与配准目标点云配准
    pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
    ndt.setTransformationEpsilon(1e-3);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(30);
    ndt.setInputSource(sampled_cloud);
    ndt.setInputTarget(target_cloud);
    ndt.align(output_cloud, key_frame->getPose());

    //获取回环位姿与回环精度
    Eigen::Matrix4f loop_pose = ndt.getFinalTransformation();
    bool converged = ndt.hasConverged();
    double fitness_score = ndt.getFitnessScore();
    int final_num_iteration = ndt.getFinalNumIteration();

    std::cout << "Loop registration fitness_score " << fitness_score << std::endl;

    //如果约束已经收敛且回环精度达到要求
    if(converged && fitness_score < loop_min_fitness_score_)
    {
      //以当前位姿建立关键帧，就近加入位姿因子图
      KeyFrame::Ptr closest_keyframe = getClosestKeyFrame(key_frame, chain);
      addEdge(key_frame, loop_pose,
              closest_keyframe, closest_keyframe->getPose(),
              Eigen::Matrix<double, 6, 6>::Identity());
      loop_constraint_count_++;
      optimize_time_ = std::chrono::steady_clock::now();
      std::cout << "Add loop constraint." << std::endl;
    }
  }
}

bool MapBuilder::needOptimize()
{//如果回环次数达到要求，算作回环
  if(loop_constraint_count_ > optimize_every_n_constraint_)
  {
    return true;
  }

  //如果有检测到回环，但回环次数不够
  if(loop_constraint_count_ > 0)
  {
    //如果本次回环时间超过十秒，也算作回环
    auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(
                            std::chrono::steady_clock::now() - optimize_time_);
    if(delta_t.count() > 10.0)
    {
      return true;
    }
  }

  //否则不回环
  return false;
}

void MapBuilder::addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud)
{//为地图添加的新的点云
  //定义不会中断的时间戳与点云计数器
  auto t1 = std::chrono::steady_clock::now();
  sequence_num_++;

  //如果是第一个点云，则需要初始化第一个子图
  if(first_point_cloud_)
  {
    //只需要处理一次
    first_point_cloud_ = false;
    //地图点云及子图只含有当前点云
    map_ += *point_cloud;
    submap_.push_back(point_cloud);
    //ndt匹配的目标只有当前点云
    ndt_.setInputTarget(point_cloud);

    //初始化第一个子图/关键帧，位姿、点云等均唯一
    KeyFrame::Ptr key_frame(new KeyFrame());
    key_frame->setId(key_frames_.size());
    key_frame->setPose(pose_);
    key_frame->setCloud(point_cloud);
    key_frames_.push_back(key_frame);

    //更新位姿因子图
    addVertex(key_frame);
    std::cout << "\033[1m\033[32m" << "------ Insert keyframe " << key_frames_.size() << " ------" << std::endl;
    return;
  }

  //不是第一个点云，则需要ndt匹配后把点云添加进去
  //为了加速ndt匹配，下采样原始点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  downSample(point_cloud, sampled_cloud);

  //设置ndt参数并配准（与上一帧）
  pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
  ndt_.setInputSource(sampled_cloud);
  ndt_.align(output_cloud, pose_);

  //获取配准结果（与上一帧）
  pose_ = ndt_.getFinalTransformation();

  //获取配准评分（与上一帧）
  bool converged = ndt_.hasConverged();
  double fitness_score = ndt_.getFitnessScore();
  int final_num_iteration = ndt_.getFinalNumIteration();

  //不收敛则警告（只是提示，没有做异常处理）
  if(!converged)
  {
    ROS_WARN("NDT does not converge!!!");
  }

  //计算配准后激光里程计的位移
  float delta = sqrt(square(pose_(0, 3) - last_update_pose_(0, 3)) +
                     square(pose_(1, 3) - last_update_pose_(1, 3)));

  //如果激光里程计位移符合建立新子图/关键帧的要求
  if(delta > map_update_distance_)
  {
    //保存当前位姿
    last_update_pose_ = pose_;

    //使用当前位姿、点云创建新的位姿因子图
    KeyFrame::Ptr key_frame(new KeyFrame());
    key_frame->setId(key_frames_.size());
    key_frame->setPose(pose_);
    key_frame->setCloud(point_cloud);
    key_frames_.push_back(key_frame);

    //为新的因子图添加节点并计算约束
    addVertex(key_frame);
    addEdge(key_frame, pose_,
            key_frames_[key_frame->getId() - 1], key_frames_[key_frame->getId() - 1]->getPose(),
        Eigen::Matrix<double, 6, 6>::Identity());

    std::cout << "\033[1m\033[32m" << "------ Insert keyframe " << key_frames_.size() << " ------" << std::endl;

    //检测因子图之间是否有闭环
    detectLoopClosure(key_frame);

    //如果有闭环且允许优化（开关），则进行全部因子图的优化
    if(enable_optimize_ && needOptimize())
    {
      //优化
      doPoseOptimize();
    }
    else
    {//不需要优化时，将点云按照配准结果变换并添加至子图
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*point_cloud, *transformed_cloud, pose_);
      submap_.push_back(transformed_cloud);

      //如果已有子图过大，则清理子图
      while(submap_.size() > submap_size_)
      {
        submap_.erase(submap_.begin());
      }

      //更新整体地图
      std::unique_lock<std::mutex> locker(map_mutex_);
      map_ += *transformed_cloud;
    }

    //把子图的所有点云累加起来作为下一次配准的目标
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i = 0; i < submap_.size(); ++i)
    {
      *target_cloud += *submap_[i];
    }

    ndt_.setInputTarget(target_cloud);
  }

  //输出提示信息
  auto t2 = std::chrono::steady_clock::now();
  auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

  std::cout << "-------------------------------------" << std::endl;
  std::cout << "Sequence number: " << sequence_num_ << std::endl;
  std::cout << "Map size: " << map_.size() << " points." << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Cost time: " << delta_t.count() * 1000.0 << "ms." << std::endl;
  std::cout << "-------------------------------------" << std::endl;
}

void MapBuilder::updateMap()
{
  //清空已有子图和地图
  std::cout << "Start update map ..." << std::endl;
  std::unique_lock<std::mutex> lock(map_mutex_);
  map_.clear();
  submap_.clear();

  //获取关键帧总数并遍历
  int n = key_frames_.size();
  for(int i = 0; i < n; ++i)
  {
    //计算相对于起点的关键帧点云的位姿并累加
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*(key_frames_[i]->getCloud()), *transformed_cloud, key_frames_[i]->getPose());
    map_ += *transformed_cloud;

    //如果是最后的关键帧，需要把它添加入子图，作为配准及回环的目标点云使用
    if(i > n - submap_size_)
    {
      submap_.push_back(transformed_cloud);
    }
  }
  std::cout << "Finish update map." << std::endl;
}

void MapBuilder::doPoseOptimize()
{
  //将第一个顶点设置为固定(起点)并初始化
  g2o::OptimizableGraph::Vertex* v = optimizer_.vertex(0);
  v->setFixed(true);

  optimizer_.initializeOptimization();

  //定义时间戳与优化迭代次数
  double chi2 = optimizer_.chi2();
  auto t1 = std::chrono::steady_clock::now();
  int iter = optimizer_.optimize(100);
  auto t2 = std::chrono::steady_clock::now();
  auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

  //判断迭代器初始化是否完成
  if (iter > 0)
  {//优化完成
    std::cout << "Optimization finished after " << iter << " iterations. Cost time " <<
                 delta_t.count() * 1000.0 << "ms." << std::endl;
    std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer_.chi2() << std::endl;
  }
  else
  {//优化失败
    std::cout << "Optimization failed, result might be invalid!" << std::endl;
    return;
  }

  //遍历所有顶点
  for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it)
  {
    //获取对应顶点信息，并为每个对应关键帧设置优化后的位姿
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(it->second);
    key_frames_[v->id()]->setPose(v->estimate().matrix().cast<float>());
  }

  //保存优化耗时，清空约束对应数量
  optimize_time_ = std::chrono::steady_clock::now();
  loop_constraint_count_ = 0;

  //更新地图
  updateMap();

  //获取当前位姿
  pose_ = key_frames_.back()->getPose();
}

void MapBuilder::getPoseGraph(std::vector<Eigen::Vector3d>& nodes,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges)
{//获取因子图信息
  //遍历所有顶点，保存位置???
  for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it)
  {
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(it->second);
    Eigen::Vector3d pt = v->estimate().translation();
    nodes.push_back(pt);
  }

  //遍历所有边，保存姿态???
  for(g2o::SparseOptimizer::EdgeSet::iterator it = optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it)
  {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(*it);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(e->vertices()[0]);
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(e->vertices()[1]);
    Eigen::Vector3d pt1 = v1->estimate().translation();
    Eigen::Vector3d pt2 = v2->estimate().translation();
    edges.push_back(std::make_pair(pt1, pt2));
  }
}

} // namespace lidar_slam_3d
