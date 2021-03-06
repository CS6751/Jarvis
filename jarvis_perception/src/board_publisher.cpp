// Backup publisher for the board because kinect info is super sketchy
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <jarvis_perception/GraspBox.h>
#include <jarvis_perception/GraspArray.h>

float BOARD_X = 0.0015;
float BOARD_Y = 0.1015;
float BOARD_Z = 0.1395;
float CENTER_X = 0.015;
float CENTER_Y = BOARD_Y/2-0.01;
float CENTER_Z = 0.025-BOARD_Z/2;
int NUM_GRIPS = 4;
double ROLLS[4] ={90,180,180,-90};
double PITCHES[4]={0,0,0,0};
double YAWS[4]={0,0,0,0};
float X[4]={CENTER_X,CENTER_X,CENTER_X,CENTER_X};
float Y[4]={CENTER_Y-BOARD_Y/2,CENTER_Y-BOARD_Y/6,
		      CENTER_Y+BOARD_Y/6, CENTER_Y+BOARD_Y/2};
float Z[4]={CENTER_Z, CENTER_Z-BOARD_Z/2,
			CENTER_Z-BOARD_Z/2, CENTER_Z};
float WEIGHTS[4] = {0.1, 0.5, 0.5, 0.9};




class BoardGenerator
{
    public:
	BoardGenerator():r_(1)
	{
	    pub_ = n_.advertise<visualization_msgs::Marker>("board",1);
	    for (int i = 0; i < 4; ++i)
	    {
		tf::Quaternion q;
		q.setRPY(ROLLS[i],PITCHES[i],YAWS[i]);
		geometry_msgs::Quaternion q2;
		q2.x = double(q.x());
		q2.y = double(q.y());
		q2.z = double(q.z());
		q2.w = double(q.w());
		quats[i] = q2;
		geometry_msgs::Point p;
		p.x = X[i];
		p.y = Y[i];
		p.z = Z[i];
		points[i] = p;
	    }
	    
	};
    void genGrips()
    {
	jarvis_perception::GraspArray all_grasps;
	all_grasps.header.frame_id = "/board_tf";
	all_grasps.header.stamp = ros::Time::now();
	jarvis_perception::GraspBox grasp_array[4];
	// generate grasps and put them in a grasp box
	for (int i = 0; i < 4; ++i)
	{
	    jarvis_perception::GraspBox grasp;
	    grasp.point = points[i];
	    grasp.orientation = quats[i];
	    grasp.weight = WEIGHTS[i];
	    grasp_array[i] = grasp;
	}

    }

	
    void run()
    {
	ROS_INFO("Invoked Run");
	while (n_.ok())
	{
	////////////////////////////
	// Generate board   
	///////////////////////////
	//ROS_INFO("Generating board");	
	visualization_msgs::Marker board;
	 uint32_t shape = visualization_msgs::Marker::CUBE;
	board.ns = "basic_shapes";
	board.header.frame_id = "/board_tf";
	board.header.stamp = ros::Time::now();
	board.id = 0;
	board.type = shape;
	board.action = visualization_msgs::Marker::ADD;
	board.lifetime = ros::Duration();
	board.scale.x = BOARD_X;
	board.scale.y = BOARD_Y;
	board.scale.z = BOARD_Z;
	board.pose.position.x = CENTER_X;
	board.pose.position.y = CENTER_Y;
	board.pose.position.z = CENTER_Z;
	board.color.r = 0.0f;
	board.color.g = 1.0f;
	board.color.b = 0.0f;
	board.color.a = 1.0;

	pub_.publish(board);
	r_.sleep();
	}
    };
    private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
//	ros::TransformListener tf_;
	ros::Rate r_;
	geometry_msgs::Quaternion quats[4];
	geometry_msgs::Point points[4];
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"board_generator");
    ROS_INFO("board generator started");
    BoardGenerator bd;
    bd.run();
    ros::spin();
}
