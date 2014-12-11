#include "ros/ros.h"
#include <ros/console.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <std_msgs/String.h>
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
//Includes for euclidean segmenting
#include <pcl/segmentation/extract_clusters.h>

//Includes for filtering
#include <pcl/filters/extract_indices.h>

class CloudDrawer
{
    public:
	CloudDrawer() : tf_(),
	target_frame_("target"),
	coefficients_(new pcl::ModelCoefficients),
	inliers_(new pcl::PointIndices)
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
	pcl::ModelCoefficients::Ptr coefficients_;
	pcl::PointIndices::Ptr inliers_;
	pcl::SACSegmentation<pcl::PointXYZ> seg_;
	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr_;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr2_;
	void euclideanSegment (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		std::vector<pcl::PointIndices> &cluster_indices)
	{
	//    FPS_CALC_BEGIN;
	    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

	    ec.setClusterTolerance (0.01); // 2cm
	    ec.setMinClusterSize (50);
	    ec.setMaxClusterSize (400);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (cloud);
	    ec.extract (cluster_indices);
	  //  FPS_CALC_END("euclideanSegmentation");
	}

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

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr filt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    /////////////////////
	// Find a plane in the target point cloud 
	// //////////////////////////
	    
	    seg_.setOptimizeCoefficients (true);
	seg_.setModelType(pcl::SACMODEL_PLANE);
	seg_.setMethodType(pcl::SAC_RANSAC);
	seg_.setDistanceThreshold(0.005);
	seg_.setInputCloud(pass_cloud);
	seg_.segment (*inliers_, *coefficients_);
	if (inliers_->indices.size () == 0)
	{
	    ROS_INFO ("Could not estimate a planar model for the given dataset.");
	}
	std::stringstream out;
	out << "Model coefficients: " << coefficients_->values[0] << " " 
	    << coefficients_->values[1] << " "
	    << coefficients_->values[2] << " " 
	    << coefficients_->values[3] << std::endl;
	std_msgs::String msg;
	msg.data = out.str();
	ROS_INFO("%s",msg.data.c_str());

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(pass_cloud);
	extract.setIndices(inliers_);
	extract.setNegative(false);
	extract.filter(*filt_cloud);
	/////////////////////
	// Euclidean Segment the cloud
	///////////////////
	/*
	std::vector<pcl::PointIndices> cluster_indices;
	euclideanSegment (pass_cloud, cluster_indices);
	
	if (cluster_indices.size() > 0)
	{
	pcl::PointIndices filt_indices = cluster_indices[0];
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(pass_cloud);
	//extract.setIndices(inliers_);
	extract.setIndices(filt_indices);
	extract.setNegative(true);
	extract.filter(*filt_cloud);
	}
	*/
	/////
	// Turn pass_cloud into output //
	// ///
	pcl::toROSMsg(*filt_cloud,output);

	pub_.publish(output);

	};

};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pose_drawer"); //Init ROS
ROS_INFO("segmenter started");
    CloudDrawer pd; //Construct class
    ros::spin(); // Run until interupted 
};
	
//TODO try to copy this guy
// pcl::PointIndices::Ptr filt_indices(&cluster_indices[0]);
	//////////////////////////////////////
	// Pull out the points in the plane
	//////////////////////////////////////
