#ifndef TRAVERSABILITY_EVALUATOR__TRAVERSABILITY_EVALUATOR_HPP_
#define TRAVERSABILITY_EVALUATOR__TRAVERSABILITY_EVALUATOR_HPP_

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);


ros::Publisher traversability_publisher;
ros::Subscriber normal_and_roughness_subscriber;

#endif