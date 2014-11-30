#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;


/* This is just for reference
void poseCallback(const turtlesim::PoseConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}
*/
int main(int argc, char** argv){
    ros::init(argc, argv, "kinect_broadcaster");
    

    ros::NodeHandle node;
    ROS_INFO("Started kinect_broadcaster node");
    ros::Rate rate(10.0);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q;
    q.setRPY(0,0,0);
    transform.setRotation(q);

    static tf::TransformBroadcaster world_openni;
    tf::Transform transform2;
    transform2.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q2;
    q2.setRPY(0,0,0);
    transform2.setRotation(q);

    while (node.ok()){
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","camera_link"));
	world_openni.sendTransform(tf::StampedTransform(transform2,
		    ros::Time::now(),"world","openni_depth_frame"));
	rate.sleep();
    
    }
    return 0;
};
