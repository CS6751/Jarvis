#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

// Includes for planar segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class CloudDrawer
{
    public:
	CloudDrawer() : tf_(),  target_frame_("target")
    {
	point_sub_.subscribe(n_, "/camera/depth/points", 10);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(point_sub_, tf_, target_frame_, 10);
	tf_filter_->registerCallback( boost::bind(&CloudDrawer::cloud_cb, this, _1) );
	pub_ = n_.advertise<sensor_msgs::PointCloud2> ("output" , 1);
    } ;

    private:
	message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
	ros::NodeHandle n_;
	std::string target_frame_;
	ros::Publisher pub_;
	tf::StampedTransform tf_world_;
	pcl::PCLPointCloud2 pcl_pc2_;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr_;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr2_;

	void filterPassThrough (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,  
	pcl::PointCloud<pcl::PointXYZ>	&result)
	{
	//    FPS_CALC_BEGIN;
	    pcl::PassThrough<pcl::PointXYZ> pass;
	    pass.setFilterFieldName ("z");
	    pass.setFilterLimits (-0.1, 0.1);
	    pass.setKeepOrganized (false);
	    pass.setInputCloud (cloud);
	    pass.filter (result);
	  //  FPS_CALC_END("filterPassThrough");
	}

	//  Callback to register with tf::MessageFilter to be called when transforms are available
	void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) 
	{
	    sensor_msgs::PointCloud2 output;
	    std::string world_frame = "/camera_link";
	    tf::Vector3 origin;
	    try 
	    {

		tf_.lookupTransform(world_frame,"/target",ros::Time(0),tf_world_);
		pcl_ros::transformPointCloud("/target",*input,output,tf_);
		origin = tf_world_.getOrigin();
//		std::cout << output.header.frame_id << std::endl;
	    }
	    catch (tf::TransformException &ex) 
	    {
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
	    }
	    
	    pcl::PCLPointCloud2 pcl_pc2;
	    pcl_conversions::toPCL(output,pcl_pc2);
	    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	    ///////////////////////////////////////////////////////////////
	    //  /*
	    //     * Pass through Filtering
	    //        */
	    //          ////////////////////////////////////////////////////
	    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::PassThrough<pcl::PointXYZ> pass;
	    pass.setInputCloud(temp_cloud);
	    pass.setFilterFieldName("z");
	    pass.setFilterLimits(-0.1,0.1);
	    pass.filter(*pass_cloud);
	    pass.setInputCloud(pass_cloud);
	    pass.setFilterFieldName("y");
	    pass.setFilterLimits(-0.1,0.1);
	    pass.filter(*pass_cloud);
      	    pass.setInputCloud(pass_cloud);
	    pass.setFilterFieldName("z");
	    pass.setFilterLimits(-0.1,0.1);
	    pass.filter(*pass_cloud);
	    /////
	    // Turn pass_cloud into output //
	    // ///
	  pcl::toROSMsg(*pass_cloud,output);
		
		pub_.publish(output);

	     };

};
	
	
	                                                                                                   int main(int argc, char ** argv)
	                                                                                                   {
	                                                                                                     ros::init(argc, argv, "pose_drawer"); //Init ROS
        CloudDrawer pd; //Construct class
	                                                                                                         ros::spin(); // Run until interupted 
	                                                                                                         };
