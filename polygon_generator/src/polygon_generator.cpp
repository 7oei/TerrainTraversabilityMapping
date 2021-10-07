#include "polygon_generator/polygon_generator.hpp"

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

void CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PointCloud<pcl::PointNormal>::Ptr traversable {new pcl::PointCloud<pcl::PointNormal>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr traversable_points {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::Normal>::Ptr traversable_normals {new pcl::PointCloud<pcl::Normal>};
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud {new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	// Convert to PCL data type
	pcl::fromROSMsg(*cloud_msg, *traversable);
    traversable_points->points.clear();
	traversable_points->points.resize(traversable->points.size());
	traversable_normals->points.clear();
	traversable_normals->points.resize(traversable->points.size());
    for(size_t i=0;i<traversable->points.size();i++){
        //normalsとpointsに分割
        traversable_points->points[i].x = traversable->points[i].x;
        traversable_points->points[i].y = traversable->points[i].y;
        traversable_points->points[i].z = traversable->points[i].z;
        traversable_normals->points[i].normal_x = traversable->points[i].normal_x;
        traversable_normals->points[i].normal_y = traversable->points[i].normal_y;
        traversable_normals->points[i].normal_z = traversable->points[i].normal_z;
    }

    ////////////////////////////////////////////////////////////////////////////////////////

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (5);
    reg.setMaxClusterSize (1000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (traversable_points);
    reg.setInputNormals (traversable_normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);  

    ////////////////////////////////////////////////////////////////////////////////////////

    colored_cloud->points.clear();
    colored_cloud->points.resize(traversable->points.size());

    colored_cloud = reg.getColoredCloud ();

    colored_cloud->header.stamp = traversable->header.stamp;
    colored_cloud->header.frame_id = traversable->header.frame_id; 

    // Convert to ROS data type
    sensor_msgs::PointCloud2 colored_pub;
    pcl::toROSMsg(*colored_cloud, colored_pub);
    // Publish the data
    pub.publish (colored_pub);
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "polygon_generator");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    sub = nh.subscribe ("traversable_cloud", 1, CloudCallback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("colored_cloud", 1);

    // Spin
    ros::spin ();
}