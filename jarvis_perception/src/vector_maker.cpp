#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>


int main( int argc, char** argv )
{
    ros::init(argc, argv, "body_generator");
    ROS_INFO("body_generator started");
    ros::NodeHandle node;
    ros::Rate r(1);
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    tf::TransformListener listener;
    tf::TransformListener world_listener;
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CYLINDER;

    while (node.ok())

    {
	visualization_msgs::Marker marker;
	//check the transform between the arm and the hand

	
	ros::Rate rate(10.0);
	
	    tf::StampedTransform transform;
	    tf::StampedTransform world_transform;
	    try{
		listener.lookupTransform("right_elbow_1","right_hand_1",
			ros::Time(0), transform);

	    }
	    catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	    }
	    try{
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

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	Eigen::Vector3f fixed_arm(0,0,1);
	Eigen::Vector3f test_hand(1,0,1);
	tf::Vector3 tf_arm = transform.getOrigin();
	tf::Vector3 tf_hand = transform.getOrigin();
	Eigen::Vector3f hand(tf_hand[0],tf_hand[1],tf_hand[2]);
	Eigen::Vector3f arm(tf_arm[0],tf_arm[1],tf_arm[2]);
	
	Eigen::Quaternionf q; 
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

	// Publish the marker
	marker_pub.publish(marker);
/*
	// Cycle between different shapes
	switch (shape)
	{
	    case visualization_msgs::Marker::CUBE:
		shape = visualization_msgs::Marker::SPHERE;
		break;
	    case visualization_msgs::Marker::SPHERE:
		shape = visualization_msgs::Marker::ARROW;
		break;
	    case visualization_msgs::Marker::ARROW:
		shape = visualization_msgs::Marker::CYLINDER;
		break;
	    case visualization_msgs::Marker::CYLINDER:
		shape = visualization_msgs::Marker::CUBE;
		break;
	}
	*/

	r.sleep();
    }
	                                                                                                                                                                                                                                 }
