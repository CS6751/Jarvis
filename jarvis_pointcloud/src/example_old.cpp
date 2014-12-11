#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Geometry>
//
ros::Publisher pub;
geometry_msgs::PointStamped center;
tf::TransformListener* listener;
// gets a spherical cloud centered on the center point 
/*
void getSphereCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &result )

{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
    pcl::PassThrough<pcl::PointXYZ> xfilter;
    xfilter.setInputCloud(cloud);
    xfilter.setFilterFieldName("x");
    xfilter.setFilterLimits(-0.1, 0.1);
    xfilter.filter(*temp_cloud);
    
    pcl::PassThrough<pcl::PointXYZ> yfilter;
    yfilter.setInputCloud(temp_cloud);
    yfilter.setFilterFieldName("y");
    yfilter.setFilterLimits(-0.1, 0.1);
    yfilter.filter(*temp_cloud);

    pcl::PassThrough<pcl::PointXYZ> zfilter;
    zfilter.setInputCloud(temp_cloud);
    zfilter.setFilterFieldName("z");
    zfilter.setFilterLimits(-0.1, 0.1);
    zfilter.filter(result);


}
*/
//
 
    void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;
    // Create a pointcloud to do processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pcl;
    // Find the center point in the point cloud frame
    sensor_msgs::PointCloud2 world_cloud;
    std::string world_frame = input->header.frame_id;
    std::cout << world_frame << std::endl;
    tf::StampedTransform world_tf;
//    try{
	//transform the point to world coordinates
	listener->lookupTransform("/camera_link",world_frame, ros::Time(0),world_tf);	
    // Convert PC2 to pointCloud
	//pcl::fromROSMsg(*input, *input_pcl);
    //
    //DO Data processing here ...
	output_pcl = input_pcl;
//Convert back to pointcloud
	//pcl::toROSMsg(*output_pcl, output);
	// Publish the data.
output = *input;
	pub.publish (output);



  //  }
    //catch (tf::TransformException ex){
//	ROS_ERROR("%s",ex.what());
//	ros::Duration(1.0).sleep();
  //  }
}

    int
main (int argc, char** argv)
{
    
    /*
    std::string topic;
    if (argc > 1)
    {
	topic = argv[0];
    }
    else
    {
	topic = "/camera/depth_registered/points";
    }
    */
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    ROS_INFO("Started looking at Pointcloud");
    // Create a ROS subscriber for the input point cloud
    std::string topic =  "/camera/depth/points";
     ros::Subscriber sub = nh.subscribe (topic, 1, cloud_cb);
    // Create a ROS publisher for the output point cloud
     tf::TransformListener lr(ros::Duration(50));
	 listener=&lr;
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}
