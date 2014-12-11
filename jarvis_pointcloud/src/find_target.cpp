#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h
class Segmenter
{
    public:
	Segmenter()
	{
	};

    private:
	ros::Subscriber sub_;
	ros::Publisher pub_;
	pcl::PCLPointCloud2 pcl_pc2_;
}
