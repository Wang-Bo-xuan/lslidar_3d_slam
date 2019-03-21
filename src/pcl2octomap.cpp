#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>

octomap_msgs::Octomap octo;
octomap::OcTree *map;
double resolution;

void PointMapCallBack(sensor_msgs::PointCloud2 msg)
{
  pcl::PointCloud<pcl::PointXYZI> pc;
  pcl::fromROSMsg(msg,pc);

  for(auto p:pc.points)
  {
      map->updateNode( octomap::point3d(p.x, p.y, p.z), true );
  }
  map->updateInnerOccupancy();

  octomap_msgs::fullMapToMsg(*map,octo);
}

bool SaveOctomapCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("save octotree map");
  map->writeBinary("/home/wangboxuan/Desktop/map.bt");

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

  map = new octomap::OcTree(resolution);

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
