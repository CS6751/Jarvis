#include "ros/package.h"
#include "geometry_msgs/Pose2D.h"
#include "jarvis_grasp/goal_pose.h"
#include "PoseGenerator.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "std_msgs/Header.h"
#include <cmath>
#include <tf/transform_broadcaster.h>

#define PI 3.1415926

int main(int argc, char **argv) {
  ros::init(argc, argv, "PoseGenerator");

  ros::NodeHandle n;

  PoseGenerator p(n);

  ros::Rate loop_rate(10);

  // Broadcast tf frame
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                        "youbot", "arm_base"));

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

PoseGenerator::PoseGenerator(ros::NodeHandle n) {

  // Set up pub for publishing goal pose
  pose_pub =  n.advertise<jarvis_grasp::goal_pose>("goal_pose", 10);

  // Set up sub for grasp points
  loc_sub = n.subscribe("grasp", 10, &PoseGenerator::loc_callback, this);

  // Set up sub for current configuration
  config_sub = n.subscribe("joint_states", 10,
                           &PoseGenerator::config_callback, this);

  // Set up sub for person position
  marker_sub = n.subscribe("visualization_marker", 10,
                           &PoseGenerator::marker_callback, this);

}

PoseGenerator::~PoseGenerator() {

}

void PoseGenerator::marker_callback(visualization_msgs::Marker mark) {
  cur_pos = mark;
}

/* This funciton receives the messages published by the youbot
   containing the joint state (angles, velocity, and "effort") and
   stores in an instance variable
*/
void PoseGenerator::config_callback(sensor_msgs::JointState config) {
  cur_state = config;
}

/* This function receives the array of potential grasp points and
   computes the best configuration to reach the optimal grasp point.
*/
void PoseGenerator::loc_callback(const
                                 jarvis_perception::GraspArray::ConstPtr& grasp_points) {

  // Select grasp with highest weight
  int max_ind = 0;
  float max_w = 0;
  for (int i = 0; i++; i < grasp_points->grasps.size()) {
    if (grasp_points->grasps[i].weight > max_w) {
      max_ind = i;
      max_w = grasp_points->grasps[i].weight;
    }
  }
  jarvis_perception::GraspBox box = grasp_points->grasps[max_ind];

  // Convert grasp point to KDL Frame
  // Create KDL vector for point. NOTE: point frame should be
  // transformed with TF first
  geometry_msgs::PointStamped p_in;
  p_in.header = grasp_points->header;
  p_in.point = box.point;
  geometry_msgs::PointStamped p_out;
  tf_listener.transformPoint("arm_base", p_in, p_out);

  // Inverse Kinematics
  geometry_msgs::Point p = p_out.point;
  float angles [5];
  float j1 [3]; // x, y, z position of shoulder relative to base
  float l1 = 1; // Length of first arm segment
  float l2 = 1; // Length of second arm segment
  angles[0] = atan2(p.y, p.x); // Base angle just points to grasp
  float x_hat = sqrt(p.x*p.x + p.y*p.y); // Projection of point on ground
  float a_ref = atan2(p.z-j1[2], x_hat-j1[0]); // Angle between j1 and goal
  angles[1] = a_ref - atan2(l2, l1);
  angles[2] = PI / 2;
  angles[3] = a_ref - angles[2];

  // Transform quaternion and calculate wrist angle
  geometry_msgs::QuaternionStamped q_in;
  q_in.header = grasp_points->header;
  q_in.quaternion = box.orientation;
  geometry_msgs::QuaternionStamped q_out;
  tf_listener.transformQuaternion("arm_base", q_in, q_out);
  // TODO: Calcualte wrist angle based on orientation

  // NOTE: For now, base movement will be ignored, set all to zero
  geometry_msgs::Pose2D base_pose;
  base_pose.x = 0;
  base_pose.y = 0;
  base_pose.theta = 0;

  // Convert angle array to vector
  std::vector<float> ang_vec;
  for (int i=0; i++; i<5) {
    ang_vec.push_back(angles[i]);
  }

  // Compose message and publish
  jarvis_grasp::goal_pose msg;
  msg.base_pose = base_pose;
  msg.joint_angles = ang_vec;
  pose_pub.publish(msg);
}
