#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h> //I believe you were using pcl/ros/conversion.h
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

    void 
cloud_cb (const sensor_msgs::PointCloud2 input)
{
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(input, pcl_pc);

    pcl::PointCloud<pcl::PointXYZ> cloud;

    pcl::fromPCLPointCloud2(pcl_pc, cloud);
//    pcl::YOUR_PCL_FUNCTION(cloud,...);
}

    int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud2_to_pcd");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("cloud", 1, cloud_cb);

    // Spin
    ros::spin ();
}
