#ifndef POLYGON_GEN__POLYGON_GEN_HPP_
#define POLYGON_GEN__POLYGON_GEN_HPP_

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


ros::Publisher pub;
ros::Subscriber sub;

#endif