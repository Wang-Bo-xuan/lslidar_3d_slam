#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>

octomap_msgs::Octomap octo;
octomap::ColorOcTree *map;
double resolution;

void PointMapCallBack(sensor_msgs::PointCloud2 msg)
{//接受到地图
  //将ros msg转换为pcl pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> pc;
  pcl::fromROSMsg(msg,pc);

  //将pcl pointcloud转换为OctoColorMap
  for(auto p:pc.points)
  {//xyz
      map->updateNode( octomap::point3d(p.x, p.y, p.z), true );
  }
  for(auto p:pc.points)
  {//rgb
      map->integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
  }
  map->updateInnerOccupancy();

  //生成地图
  octomap_msgs::fullMapToMsg(*map,octo);
}

bool SaveOctomapCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{//接受到保存地图指令
  ROS_INFO("save octotree map");
  map->write("/home/wangboxuan/Desktop/map.ot");

  return true;
}

int main(int argc,char *argv[])
{
  ros::init(argc,argv,"map_load_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate loop_rate(10.0);

  pnh.param("map_resolution",resolution,0.5);

  ros::Subscriber point_map_sub = nh.subscribe("/point_cloud_map",1,PointMapCallBack);
  ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap",10);
  ros::ServiceServer save_octo_ss  = nh.advertiseService("/save_octomap",SaveOctomapCallBack);

  map = new octomap::ColorOcTree(resolution);

  while(ros::ok())
  {
    octo.header.stamp = ros::Time::now();
    octo.header.frame_id = "map";
    octomap_pub.publish(octo);

    ros::spinOnce();
    loop_rate.sleep();
  }

  if(map)
  {
    delete map;
    map = NULL;
  }

  return 0;
}
