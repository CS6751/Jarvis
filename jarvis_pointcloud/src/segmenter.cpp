#include "ros/ros.h"
#include <ros/console.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
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

//includes for region_growing segmentation
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

//Includes for filtering
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>


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
	grip_pub_ = n_.advertise<geometry_msgs::PoseStamped> ("test_grip", 1);
    } ;

    private:
	message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;
	ros::NodeHandle n_;
	std::string target_frame_;
	ros::Publisher pub_;
	ros::Publisher grip_pub_;
	tf::StampedTransform tf_world_;
	pcl::PCLPointCloud2 pcl_pc2_;
	pcl::ModelCoefficients::Ptr coefficients_;
	pcl::PointIndices::Ptr inliers_;
	pcl::SACSegmentation<pcl::PointXYZ> seg_;

	void findNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
		pcl::PointCloud<pcl::Normal>::Ptr &normals)
	{
	    
	    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
	    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
	    normal_estimator.setSearchMethod (tree);
	    normal_estimator.setInputCloud (cloud);
	    normal_estimator.setKSearch (50);
	    normal_estimator.compute (*normals);
	}

	void regionGrowingSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		std::vector<pcl::PointIndices> cluster_indices)
	{
	    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
	    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	    findNormals(cloud,normals);
	    //Find the normals for the cloud
	    //Cluster by normals
	    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	    reg.setMinClusterSize (50);
	    reg.setMaxClusterSize (1000000);
	    reg.setSearchMethod (tree);
	    reg.setNumberOfNeighbours (30);
	    reg.setInputCloud (cloud);
	    reg.setInputNormals (normals);
	    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
	    reg.setCurvatureThreshold (1.0);
	    reg.extract (cluster_indices);
	}


	void extractSegmentCluster (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		const std::vector<pcl::PointIndices> cluster_indices,
		const int segment_index,
		pcl::PointCloud<pcl::PointXYZ> &result)
	{
	    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
	    for (size_t i = 0; i < segmented_indices.indices.size (); i++)
	    {
		pcl::PointXYZ point = cloud->points[segmented_indices.indices[i]];
		result.points.push_back (point);
	    }
	    result.width = pcl::uint32_t (result.points.size ());
	    result.height = 1;
	    result.is_dense = true;
	}

	bool checkDist(Eigen::Vector4f& pt1, Eigen::Vector4f& pt2)
	{
	    float threshold = 0.05;
	    Eigen::Vector4f diff = pt1 - pt2;
	    if (diff.norm() < threshold) return true;
	    else return false;
	}
	
	void euclideanSegment (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		std::vector<pcl::PointIndices> &cluster_indices)
	{
	    //    FPS_CALC_BEGIN;
	    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

	    ec.setClusterTolerance (0.02); // 2cm
	    ec.setMinClusterSize (50);
	    ec.setMaxClusterSize (4000);
	    ec.setSearchMethod (tree);
	    ec.setInputCloud (cloud);
	    ec.extract (cluster_indices);
	    //  FPS_CALC_END("euclideanSegmentation");
	}
	
	   void closestPt(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointXYZ target_point, std::vector<int> nn_indices)
	   {
	//use a K-d tree to find the nearest point to a designated point
	pcl::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	std::vector<float> nn_dists(1);
	tree->nearestKSearch(target_point, 1, nn_indices, nn_dists);
	}
	 
	void getOriginPoint (std::string frame, Eigen::Vector3f& point)
	{
	    geometry_msgs::PointStamped origin;
	    origin.point.x = 0;origin.point.y= 0; origin.point.z=0;
	    origin.header.frame_id = frame;
	    geometry_msgs::PointStamped tf_origin;
	    try 
	    {
		tf_.transformPoint("/target", origin, tf_origin);
	    }
	    catch (tf::TransformException &ex) 
	    {
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
	    }
	   // turn the geometry msgs to vector3f
	    point[0] =tf_origin.point.x;
	    point[1] =tf_origin.point.y;
	    point[2] = tf_origin.point.x;

	}

	void distToTransform (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string frame, Eigen::Vector4f& max_pt)
	{
	    geometry_msgs::PointStamped origin;
	    origin.point.x = 0;origin.point.y= 0; origin.point.z=0;
	    origin.header.frame_id = frame;
	    geometry_msgs::PointStamped tf_origin;
	    try 
	    {
		tf_.transformPoint("/target", origin, tf_origin);
	    }
	    catch (tf::TransformException &ex) 
	    {
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
	    }
	   // turn the geometry msgs to vector4f
	    Eigen::Vector4f vec_origin(tf_origin.point.x,tf_origin.point.y,tf_origin.point.x,0);
// Find the point with the max distance to both the hand and the arm 
		pcl::getMaxDistance(*cloud, vec_origin, max_pt);

	};



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
		    //transform pointcloud into target frame so useful passthrough filtering can happen
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
		    /* DEBUGGING FOR PLANAR SEGMENTATION
		    
		       std::stringstream out;
		    out << "Model coefficients: " << coefficients_->values[0] << " " 
			<< coefficients_->values[1] << " "
			<< coefficients_->values[2] << " " 
			<< coefficients_->values[3] << std::endl;
		    std_msgs::String msg;
		    msg.data = out.str();
		    
		    ROS_INFO("%s",msg.data.c_str());
		    */
		    pcl::ExtractIndices<pcl::PointXYZ> extract;
		    extract.setInputCloud(pass_cloud);
		    extract.setIndices(inliers_);
		    extract.setNegative(false);
		    extract.filter(*filt_cloud);
		       std::vector<pcl::PointIndices> cluster_indices;
		    pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		    /////////////////////
		    // Euclidean Segment the cloud
		    ///////////////////
		    /* This doesn't do anything useful
		       euclideanSegment (pass_cloud, cluster_indices);
//Grab the biggest segment
		       if (cluster_indices.size() > 0)
		       {
			  extractSegmentCluster (filt_cloud, cluster_indices,0,*seg_cloud);
			  filt_cloud.swap(seg_cloud);
			  ROS_INFO("euclidean segmentation successful");
		       }
*/
		    ///////////////
      ////////////////////////
      // Normal Segment the cloud
      //
      regionGrowingSegmentation(filt_cloud, cluster_indices);
      if (cluster_indices.size() > 0)
      {
	  extractSegmentCluster (filt_cloud, cluster_indices,0,*seg_cloud);
	  filt_cloud.swap(seg_cloud);
	  ROS_INFO("region growing successful");
      }


      // Get points that are the farthest from the hand and the arm
      // /////////////////////////
      /////
      Eigen::Vector4f hand_far;
      Eigen::Vector4f arm_far;



      distToTransform (filt_cloud, "right_hand_1", hand_far);
      distToTransform (filt_cloud, "right_elbow_1", arm_far);

      /* DEBUG FOR FAR POINTS FROM HAND AND ARM
	 std::stringstream out;
	 out << "Far pt Hand: " << hand_far[0] << " " 
	 << hand_far[1] << " "
			<< hand_far[2] << " " << std::endl
			
		<< "Far pt arm: " << arm_far[0] << " " 
			<< arm_far[1] << " "
			<< arm_far[2] << " " 
			<< std::endl;
		    std_msgs::String msg;
		    msg.data = out.str();
		    ROS_INFO("%s",msg.data.c_str());
		    */
/* DEBUG FOR HAND ARM AGREEMENT	
      std::stringstream out;
	if (checkDist(hand_far, arm_far))
	{
	    out << "hand - arm agreement" << std::endl;
	}
	else
	{
	    out << "NO hand-arm agreement" << std::endl;
	}
	std_msgs::String msg;
	msg.data = out.str();
	ROS_INFO("%s",msg.data.c_str());

	*/
	// Turn pass_cloud into output //
	// ///
	//
//////////////////////////////////
/// Find the vector from the elbow to the hand
      Eigen::Vector3f elbow;
      Eigen::Vector3f hand;
      Eigen::Vector3f eh = hand-elbow; //vector from elbow to hand
      getOriginPoint ("right_hand_1", hand);
      getOriginPoint ("right_elbow_1",elbow);
      //Project the elbow-hand vector into the plane
      Eigen::Vector3f plane_coeffs(coefficients_->values[0],
	      coefficients_->values[1], 
	      coefficients_->values[2]);
	      

      Eigen::Vector3f b_hat = plane_coeffs/plane_coeffs.norm();
      Eigen::Vector3f eh_plane = eh - eh.dot(b_hat)*b_hat;

// Hand Point
      std::stringstream out;
	    out << "Hand: (" << hand[0]<< " " << hand[1]<< " " <<hand[2]<< ")" <<std::endl
		<<" eh_plane: (" << eh_plane[0] << " " << eh_plane[1] << " " << eh_plane[2] << ")" 
		<<std::endl;

	std_msgs::String msg;
	msg.data = out.str();
     ROS_INFO("%s",msg.data.c_str()); 
      
     if (eh_plane.norm() > 1e-15)
     {
     eh_plane = eh_plane/eh_plane.norm();	
	std::vector<int> close_index(1);
      pcl::PointXYZ *target_point(new pcl::PointXYZ(eh_plane[0]+hand[0],
		  eh_plane[1]+hand[1],
		  eh_plane[2]+hand[2]));
      // Find the closest point beyond that extended vector 
      closestPt(filt_cloud, *target_point, close_index);
      if (close_index.size() > 0)
      {
	  geometry_msgs::PoseStamped grip_pt;
	  grip_pt.pose.position.x = filt_cloud->points[close_index[0]].x;
	  grip_pt.pose.position.y = filt_cloud->points[close_index[0]].y;
	  grip_pt.pose.position.z = filt_cloud->points[close_index[0]].z;
	  grip_pt.header.frame_id = "/target";
	  // find the normal to the plane at that point
	  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	  findNormals(filt_cloud, normals);

	  // let the z axis of the grip be the eh_plane vector, x is normal to plane vector
	  Eigen::Vector3f grip_x(normals->points[close_index[0]].normal_x,
		  normals->points[close_index[0]].normal_y,
		  normals->points[close_index[0]].normal_z);

	  Eigen::Vector3f grip_y = eh_plane.cross(grip_x);
	  Eigen::Matrix3f mat;
	  mat.col(0) = grip_x;
	  mat.col(1) = grip_y;
	  mat.col(2) = eh_plane;
	  Eigen::Quaternionf q(mat);
	  grip_pt.pose.orientation.x = q.x();
	  grip_pt.pose.orientation.y = q.y();
	  grip_pt.pose.orientation.z = q.z();
	  grip_pt.pose.orientation.w = q.w();

	  grip_pub_.publish(grip_pt);
      }
      else
      {
	  ROS_INFO("No closest point");
      }
     }


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
