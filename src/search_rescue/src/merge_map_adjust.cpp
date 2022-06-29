#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>

nav_msgs::OccupancyGrid merged_map;


void merged_map_callback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  merged_map.header = msg->header;
  merged_map.info = msg->info;
  merged_map.data = msg->data;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "merge_map_adjust");
  ros::NodeHandle merge_map;
  const auto adjusted_map_pub = merge_map.advertise<nav_msgs::OccupancyGrid>("/map_adjusted", 100);
  const auto merged_map_sub = merge_map.subscribe<nav_msgs::OccupancyGrid>("/map", 100, merged_map_callback);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if ( (merged_map.data.size() != 0) || merged_map.info.origin.position.x !=0)
    {
      nav_msgs::OccupancyGrid adjusted_map;
      adjusted_map.header.frame_id = "map";
      adjusted_map.info.resolution = 0.05;
      adjusted_map.info.origin.position.x =  -10.0;
      adjusted_map.info.origin.position.y = -10.0;
      adjusted_map.info.origin.position.z = 0.0;
      adjusted_map.info.origin.orientation.w = 0.0;
      adjusted_map.info.width = merged_map.info.width;
      adjusted_map.info.height = merged_map.info.height;
      adjusted_map.data = merged_map.data;
      adjusted_map_pub.publish(adjusted_map);
    }

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}