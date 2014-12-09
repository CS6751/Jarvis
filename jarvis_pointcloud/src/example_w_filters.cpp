#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"


class CloudDrawer
{
    public:
	PoseDrawer() : tf_(),  target_frame_("target")
    {
	point_sub_.subscribe(n_, "/camera/depth/points", 10);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(point_sub_, tf_, target_frame_, 10);
	tf_filter_->registerCallback( boost::bind(&CloudDrawer::cloud_cb, this, _1) );
	pub_ = n_.advertise<sensor_msgs::PointCloud2> ("output" , 1);
    } ;

    private:
	message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
	ros::NodeHandle n_;
	std::string target_frame_;
	ros::Publisher pub_;

	//  Callback to register with tf::MessageFilter to be called when transforms are available
	void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) 
	{
	    sensor_msgs::PointCloud2 output;
	    try 
	    {
		tf_.transformPointCloud(target_frame_, *input, output);
		pub_.publish(output);
/*
		printf("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
			point_out.point.x,
			point_out.point.y,
			point_out.point.z);
*/
			}
	    catch (tf::TransformException &ex) 
	    {
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
	    }
	};

};
	
	
	                                                                                                   int main(int argc, char ** argv)
	                                                                                                   {
	                                                                                                     ros::init(argc, argv, "pose_drawer"); //Init ROS
	                                                                                                       PoseDrawer pd; //Construct class
	                                                                                                         ros::spin(); // Run until interupted 
	                                                                                                         };
