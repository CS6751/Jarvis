#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

/* This node broadcasts a cylindar around a perceived arm and the center of a sphere around the hand/object to be detected
 */

int main( int argc, char** argv )
{
    ros::init(argc, argv, "body_generator");
    ROS_INFO("body_generator started");
    ros::NodeHandle node;
    ros::Rate r(1);
    
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher sphere_pub = node.advertise<geometry_msgs::PointStamped>("sphere_center",1);
    // TF Transforms.
    tf::TransformListener listener;//transform between hand and elbow
    tf::TransformListener world_listener; //transform between elbow and world
    tf::TransformBroadcaster broadcaster;
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    ros::Rate loop_rate(10);
    while (ros::ok())

    {
	visualization_msgs::Marker marker; //marker for the arm 
	//check the transform between the arm and the hand

	tf::StampedTransform transform;
	tf::StampedTransform world_transform;
	try{
	    listener.lookupTransform("right_elbow_1","right_hand_1",
		    ros::Time(0), transform);
	    listener.lookupTransform("world","right_elbow_1",
			ros::Time(0), world_transform);

	}
	catch (tf::TransformException ex){
	    ROS_ERROR("%s",ex.what());
	    ros::Duration(1.0).sleep();
	}
	     

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/right_elbow_1";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	Eigen::Vector3d fixed_arm(0,0,1);
	Eigen::Vector3f test_hand(1,0,1);
	tf::Vector3 tf_arm = transform.getOrigin();
	tf::Vector3 tf_hand = transform.getOrigin();
	Eigen::Vector3d hand(tf_hand[0],tf_hand[1],tf_hand[2]);
	Eigen::Vector3d arm(tf_arm[0],tf_arm[1],tf_arm[2]);
	
	Eigen::Quaterniond q; 
	  q  = q.FromTwoVectors(fixed_arm,hand);
	
	marker.pose.position.x = hand.x()/2;
	marker.pose.position.y = hand.y()/2;
	marker.pose.position.z = hand.z()/2;
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();
	marker.pose.orientation.z = q.z();
	marker.pose.orientation.w = q.w();

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = hand.norm();

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	//Set up the sphere center
	// IN point form
	geometry_msgs::PointStamped center; // center of the sphere
	float dist_to_center = 0.1;
	center.header.frame_id = "/right_elbow_1";
	center.header.stamp = ros::Time::now();
	Eigen::Vector3d center_v = hand + 0.1*hand/hand.norm();
	center.point.x = center_v[0];
	center.point.y = center_v[1];
	center.point.z = center_v[2];
	// In coordinate form
	tf::Transform center_transform;
	tf::Vector3 center_tf;
	tf::vectorEigenToTF(center_v,center_tf);
	center_transform.setOrigin(center_tf);




	broadcaster.sendTransform(tf::StampedTransform(center_transform, 
		    ros::Time::now(),"/right_elbow_1","/target"));
	// Publish the marker
	marker_pub.publish(marker);
	sphere_pub.publish(center);

	r.sleep();
    }
	                                                                                                                                                                                                                                 }
