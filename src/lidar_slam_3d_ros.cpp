#include "lidar_slam_3d_ros.h"
#include "geo_transform.h"
#include <pcl_conversions/pcl_conversions.h>

LidarSlam3dRos::LidarSlam3dRos()
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string point_cloud_topic, gps_topic;

  //获取参数服务器参数
  private_nh.param("base_frame", base_frame_, std::string("base_link"));
  private_nh.param("map_frame", map_frame_, std::string("map"));
  private_nh.param("publish_freq", publish_freq_, 0.2);
  private_nh.param("point_cloud_topic", point_cloud_topic, std::string("velodyne_points"));
  private_nh.param("gps_topic", gps_topic, std::string("fix"));
  private_nh.param("min_scan_distance", min_scan_distance_, 2.0);
  private_nh.param("max_scan_distance", max_scan_distance_, 10.0);
  private_nh.param("enable_floor_filter", enable_floor_filter_, false);

  //绑定发布订阅,并开启保存地图服务
  map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 1, true);
  path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);
  gps_path_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 1, true);
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);
  filtered_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1, true);
  floor_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("floor_points", 1, true);
  constraint_list_pub_ = nh.advertise<visualization_msgs::MarkerArray>("constraint_list", 1, true);

  optimization_srv_ = nh.advertiseService("optimization", &LidarSlam3dRos::optimizationCallback, this);
  save_pcd_srv_ = nh.advertiseService("/save_pcdmap",&LidarSlam3dRos::savePCDCallBack,this);

  point_cloud_sub_ = nh.subscribe(point_cloud_topic, 10000, &LidarSlam3dRos::pointCloudCallback, this);
  gps_sub_ = nh.subscribe(gps_topic, 100, &LidarSlam3dRos::gpsCallback, this);

  //如果没有获取激光数据，则不发送地图及约束
  this->get_lidar_ = false;
  publish_thread_.reset(new std::thread(std::bind(&LidarSlam3dRos::publishLoop, this)));
}

Vector6f LidarSlam3dRos::getPose(const Eigen::Matrix4f& T)
{//获取位姿
  Vector6f pose;
  //从t中保存位置信息
  pose(0) = T(0, 3);
  pose(1) = T(1, 3);
  pose(2) = T(2, 3);

  //从R中保存姿态信息
  tf::Matrix3x3 R;
  double roll, pitch, yaw;
  R.setValue(T(0, 0), T(0, 1), T(0, 2),
             T(1, 0), T(1, 1), T(1, 2),
             T(2, 0), T(2, 1), T(2, 2));
  R.getRPY(roll, pitch, yaw);
  pose(3) = roll;
  pose(4) = pitch;
  pose(5) = yaw;

  return pose;
}

bool LidarSlam3dRos::optimizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{//优化已有位姿约束，优化暂时由两种方式启动：a.子图检测到有效回环 b.手动启动 rosservice call /optimization "{}"
  map_builder_.doPoseOptimize();

  return true;
}

bool LidarSlam3dRos::savePCDCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{//保存pcd地图服务，暂时只能手动启动 rosserver call /save_pcdmap "{}"
  ROS_INFO("save pcd map");

  //获取地图，将地图转换为pcl::PointCloud<pcl::PointXYZRGB>格式并保存为pcd
  sensor_msgs::PointCloud2 map_msg;
  pcl::PCLPointCloud2 pcl_pc2;
  map_builder_.getMap(map_msg);
  pcl_conversions::toPCL(map_msg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
  pcl::fromPCLPointCloud2(pcl_pc2,temp_cloud);
  pcl::io::savePCDFileASCII("/home/wangboxuan/Desktop/map.pcd",temp_cloud);

  return true;
}

void LidarSlam3dRos::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{//获取gps数据
  //gps失效(多楼、地下等)
  if(gps_msg->status.status == -1)
  {
    ROS_WARN("Lost Gps!!!");

    return;
  }

  //gps地理坐标系变换
  double x, y;
  lidar_slam_3d::WGS84ToUTM(gps_msg->latitude, gps_msg->longitude, x, y);

  Eigen::Vector3d pose(x, y, 0.0);

  //如果gps自启动都没有获取到有效数据，则抛弃当前gps数据
  if(!gps_origin_)
  {
    gps_origin_ = pose;
  }

  //gps变化量
  pose -= *gps_origin_;

  //发布topic
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = gps_msg->header.stamp;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose.position.x = pose(0);
  pose_msg.pose.position.y = pose(1);
  pose_msg.pose.position.z = pose(2);

  tf::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  gps_path_msg_.poses.push_back(pose_msg);

  gps_path_msg_.header.stamp = gps_msg->header.stamp;
  gps_path_msg_.header.frame_id = map_frame_;
  gps_path_pub_.publish(gps_path_msg_);
}

void LidarSlam3dRos::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{//获取激光点云
  //成功获取点云则可以发布地图及约束
  this->get_lidar_ = true;

  //ros msg转换为pcl变量
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

  //半径滤波，不在距离范围均不选用
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clipped_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for(const pcl::PointXYZRGB& point: point_cloud->points)
  {
    double r = sqrt(square(point.x) + square(point.y));
    if (r >= min_scan_distance_ && r <= max_scan_distance_)
    {
      clipped_point_cloud->push_back(point);
    }
  }

  if(enable_floor_filter_)
  {//如果需要拟合地面，则使用拟合并去除掉地面后的点云建图
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    floor_filter_.filter(clipped_point_cloud, filtered_point_cloud, floor_point_cloud);
    map_builder_.addPointCloud(filtered_point_cloud);

    //发布地面点云
    sensor_msgs::PointCloud2 floor_cloud_msg;
    pcl::toROSMsg(*floor_point_cloud, floor_cloud_msg);
    floor_cloud_msg.header.stamp = ros::Time::now();
    floor_cloud_msg.header.frame_id = point_cloud_msg->header.frame_id;
    floor_points_pub_.publish(floor_cloud_msg);

    //发布无地面点云
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_point_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header.stamp = ros::Time::now();
    filtered_cloud_msg.header.frame_id = point_cloud_msg->header.frame_id;
    filtered_point_cloud_pub_.publish(filtered_cloud_msg);
  }
  else
  {//如果不需要拟合地面，直接将点云添加入已有地图
    map_builder_.addPointCloud(clipped_point_cloud);
  }

  //获取当前地图位姿并发布基础信息
  Vector6f pose = getPose(map_builder_.getTransformation());
  publishPose(pose, point_cloud_msg->header.stamp);
  publishTf(pose, point_cloud_msg->header.stamp);
  publishPath(pose, point_cloud_msg->header.stamp);
}

void LidarSlam3dRos::publishPose(const Vector6f& pose, const ros::Time& t)
{//发布当前位姿
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = t;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose.position.x = pose(0);
  pose_msg.pose.position.y = pose(1);
  pose_msg.pose.position.z = pose(2);
  tf::Quaternion q;
  q.setRPY(pose(3), pose(4), pose(5));
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  pose_pub_.publish(pose_msg);
}

void LidarSlam3dRos::publishPath(const Vector6f& pose, const ros::Time& t)
{//发布激光里程计路径
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = t;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose.position.x = pose(0);
  pose_msg.pose.position.y = pose(1);
  pose_msg.pose.position.z = pose(2);

  tf::Quaternion q;
  q.setRPY(pose(3), pose(4), pose(5));
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  path_msg_.poses.push_back(pose_msg);

  path_msg_.header.stamp = t;
  path_msg_.header.frame_id = map_frame_;
  path_pub_.publish(path_msg_);
}

void LidarSlam3dRos::publishTf(const Vector6f& pose, const ros::Time& t)
{//发布tf变化
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
  tf::Quaternion q;
  q.setRPY(pose(3), pose(4), pose(5));
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, t, "map", "base_link"));
}

void LidarSlam3dRos::publishMap()
{
  //发布现有地图
  sensor_msgs::PointCloud2 map_msg;
  map_builder_.getMap(map_msg);
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = map_frame_;
  map_pub_.publish(map_msg);
}

void LidarSlam3dRos::publishConstraintList()
{//发布约束
  std::vector<Eigen::Vector3d> graph_nodes;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> graph_edges;

  //获取节点信息及约束信息
  map_builder_.getPoseGraph(graph_nodes, graph_edges);

  //填充显示消息，节点用红色球体，约束用绿色线段
  //节点信息
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0);

  int id = 0;
  for (int i = 0; i < graph_nodes.size(); ++i)
  {
    marker.id = id;
    marker.pose.position.x = graph_nodes[i](0);
    marker.pose.position.y = graph_nodes[i](1);
    marker.pose.position.z = graph_nodes[i](2);
    marker_array.markers.push_back(visualization_msgs::Marker(marker));
    id++;
  }

  //约束信息
  visualization_msgs::Marker edge;
  edge.header.frame_id = map_frame_;
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.r = 0.0;
  edge.color.g = 1.0;
  edge.color.b = 0.0;
  edge.color.a = 1.0;

  for (int i = 0; i < graph_edges.size(); ++i)
  {
    edge.points.clear();
    geometry_msgs::Point p;
    p.x = graph_edges[i].first(0);
    p.y = graph_edges[i].first(1);
    p.z = graph_edges[i].first(2);
    edge.points.push_back(p);
    p.x = graph_edges[i].second(0);
    p.y = graph_edges[i].second(1);
    p.z = graph_edges[i].second(2);
    edge.points.push_back(p);
    edge.id = id;
    marker_array.markers.push_back(visualization_msgs::Marker(edge));
    id++;
  }

  //发布
  constraint_list_pub_.publish(marker_array);
}

void LidarSlam3dRos::publishLoop()
{//如果接收到了激光点云，则以固定的频率发布地图和约束
  ros::Rate rate(publish_freq_);

  while (ros::ok())
  {
    if(this->get_lidar_)
    {
      publishMap();
      publishConstraintList();
    }

    rate.sleep();
  }
}
