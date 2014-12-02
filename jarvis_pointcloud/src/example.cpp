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
//
ros::Publisher pub;
geometry_msgs/PointStamped center;
tf::TransformListener listener;
//
void point_cb(const std_msg::PointStamped& point)
{
    center = point;
}
// gets a spherical cloud centered on the center point
void getSphereCloud()
{
    pcl::PassThrough<pcl::PointXYZ> xfilter;
    xfilter.setInputCloud(cloud);
    xfilter.setFilterFieldName("x");
    xfilter.setFilterLimits(-0.1, 0.1);
    xfilter.filter(*cloud);
    
    pcl::PassThrough<pcl::PointXYZ> yfilter;
    yfilter.setInputCloud(cloud);
    yfilter.setFilterFieldName("x");
    yfilter.setFilterLimits(-0.1, 0.1);
    yfilter.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> zfilter;
    xfilter.setInputCloud(cloud);
    xfilter.setFilterFieldName("x");
    xfilter.setFilterLimits(-0.1, 0.1);
    filter.filter(*cloud);


}
//
    void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;
    // Create a pointcloud to do processing
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Find the center point in the point cloud frame
    geometry_msgs::PointStamped c_w;
    try{
	tf::StampedTransform transform;
	listener.lookupTransform(input.header.frame_id,
		center.header.frame_id,
		ros::Time(0), transform);
	//transform the point to global coordiantes
listener.trasnformPoint(input.frame_id, center, c_w);	
	// Do data processing here...
	
    // Convert PC2 to pointCloud
    pcl::fromROSMsg(input, cloud);
    //
	output = *input;

	// Publish the data.

	pub.publish (output);



    }
    catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
    }
}

    int
main (int argc, char** argv)
{
    ROS_INFO("Started looking at Pointcloud");
    char* topic;
    if (argc > 1)
    {
	char* topic = argv[0];
    }
    else
    {
	topic = "/camera/depth_registered/points";
    }
    ROS_INFO(topic);
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe (topic, 1, cloud_cb);
    ros::Subscriber pt = nh.subscribe ("sphere_center",1,point_cb);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}
