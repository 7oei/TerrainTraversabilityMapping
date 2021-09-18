#ifndef ORGANIZED_VGF__ORGANIZED_VGF_HPP_
#define ORGANIZED_VGF__ORGANIZED_VGF_HPP_

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


ros::Publisher pub;
ros::Subscriber sub;

#endif