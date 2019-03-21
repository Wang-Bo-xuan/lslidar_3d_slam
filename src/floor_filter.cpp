#include "floor_filter.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace lidar_slam_3d
{

FloorFilter::FloorFilter() :
  height_clip_range_(2.5), floor_min_points_num_(1000), point_normal_threshhold_(20),
  floor_normal_threshhold_(20)
{

}

void FloorFilter::filter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& floor_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  //直通滤波，在z轴上划分两个范围的点云，范围:[-2.5,0]，范围内clipped_cloud，范围外filtered_cloud
  pcl::PassThrough<pcl::PointXYZRGB> pass_through_filter;
  pass_through_filter.setInputCloud(source_cloud);
  pass_through_filter.setFilterFieldName("z");
  pass_through_filter.setFilterLimits(-height_clip_range_, 0.0);
  pass_through_filter.filter(*clipped_cloud);
  pass_through_filter.setNegative(true);
  pass_through_filter.filter(*filtered_cloud);

  //计算范围内点云法向量
  pcl::NormalEstimation<pcl::PointXYZRGB,
  pcl::Normal> normal_estimator;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normal_estimator.setInputCloud(clipped_cloud);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setKSearch(20);
  normal_estimator.compute(*normals);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  horizontal_cloud->reserve(clipped_cloud->size());

  //区分角度大于point_normal_threshold的点云
  for (int i = 0; i < clipped_cloud->size(); i++)
  {
    float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
    if(std::abs(dot) > std::cos(point_normal_threshhold_ * M_PI / 180.0))
    {
      horizontal_cloud->push_back(clipped_cloud->at(i));
    }
    else
    {
      filtered_cloud->push_back(clipped_cloud->at(i));
    }
  }

  //RANSAC拟合平面，存入inliers
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_plane(
        new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(horizontal_cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_plane);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  ransac.setDistanceThreshold(0.2);
  ransac.computeModel();
  ransac.getInliers(inliers->indices);

  //如果inliers点云数量过少
  int inliers_num = inliers->indices.size();
  if(inliers->indices.size() < floor_min_points_num_)
  {
    ROS_INFO("Too less inliers. Only %d.", inliers_num);
    return;
  }

  //获取地面拟合的多项式系数
  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);

  //如果角度小于floor_normal_threshold则抛弃
  double dot = coeffs.head<3>().dot(Eigen::Vector3f::UnitZ());
  if(std::abs(dot) < std::cos(floor_normal_threshhold_ * M_PI / 180.0))
  {
    ROS_INFO("The normal is not vertical!");
    return;
  }

  //分割地面内点子集与外点子集
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(horizontal_cloud);
  extract.setIndices(inliers);
  extract.filter(*floor_cloud);
  extract.setNegative(true);
  extract.filter(*outliers_cloud);

  *filtered_cloud += *outliers_cloud;
}

} // namespace lidar_slam_3d
