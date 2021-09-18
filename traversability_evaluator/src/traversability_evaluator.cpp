#include "traversability_evaluator/traversability_evaluator.hpp"

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>


void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_and_roughness {new pcl::PointCloud<pcl::PointXYZINormal>};
	pcl::PointCloud<pcl::PointXYZI>::Ptr traversability {new pcl::PointCloud<pcl::PointXYZI>};
	// Convert to PCL data type
	pcl::fromROSMsg(*cloud_msg, *normal_and_roughness);
	traversability->points.clear();
	traversability->points.resize(normal_and_roughness->points.size());

	// #ifdef _OPENMP
	// #pragma omp parallel for
	// #endif
	for(size_t i=0;i<normal_and_roughness->points.size();i++){

        traversability->points[i].x = normal_and_roughness->points[i].x;
		traversability->points[i].y = normal_and_roughness->points[i].y;
		traversability->points[i].z = normal_and_roughness->points[i].z;
        // std::cout << (float)cos(90/180*M_PI) << std::endl;

        if(normal_and_roughness->points[i].normal_y<-cos(10.0f/180*M_PI)&&normal_and_roughness->points[i].intensity<0.001){//10degree//1cm
            traversability->points[i].intensity = FLT_MAX;
        }
        else{
            traversability->points[i].intensity = -FLT_MAX;
        }

	}
	for(size_t i=0;i<traversability->points.size();){
		if(std::isnan(traversability->points[i].intensity)){
			std::cout << "deleted NAN normal" << std::endl;
			traversability->points.erase(traversability->points.begin() + i);
		}
		else	i++;
	}

	traversability->header.stamp = normal_and_roughness->header.stamp;
	traversability->header.frame_id = normal_and_roughness->header.frame_id; 
	sensor_msgs::PointCloud2 traversability_pub;
	pcl::toROSMsg(*traversability, traversability_pub);

	// Publish the data
	traversability_publisher.publish (traversability_pub);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "traversability_evaluator");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  normal_and_roughness_subscriber = nh.subscribe ("normal_and_roughness_cloud", 1, CloudCallback);

  // Create a ROS publisher for the output point cloud
  traversability_publisher = nh.advertise<sensor_msgs::PointCloud2> ("traversability_cloud", 1);

  // Spin
  ros::spin ();
}
