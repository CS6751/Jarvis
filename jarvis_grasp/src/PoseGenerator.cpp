#include "ros/package.h"
#include "geometry_msgs/Pose2D.h"
#include "jarvis_grasp/goal_pose.h"
#include "PoseGenerator.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include <cmath>

int main(int argc, char **argv) {
  ros::init(argc, argv, "PoseGenerator");

  ros::NodeHandle n;

  PoseGenerator p(n);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

PoseGenerator::PoseGenerator(ros::NodeHandle n) {

  // Load kinematic data from URDF and use to create ik object
  loadIK();

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
  delete ik;
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

  // Select first possible grasp point (temporary)
  jarvis_perception::GraspBox box = grasp_points->grasps[0];

  // Convert grasp point to KDL Frame
  // Create KDL vector for point. NOTE: point frame should be
  // transformed with TF first
  geometry_msgs::PointStamped p_in;
  p_in.header = grasp_points->header;
  p_in.point = box.point;
  geometry_msgs::PointStamped p_out;
  tf_listener.transformPoint("/arm_base", p_in, p_out);

  // Vector stores possible configurations that solve IK
  std::vector<KDL::JntArray> q_out;

  if (!ik->CartToJnt(cur_q, p_frame, q_out)) {
    ROS_INFO("Failed to solve for joint configuration using IK");
    return;
  }

  // Choose best configuration.
  // NOTE: This should be done using highest weight, but just choose
  // the first option now
  KDL::JntArray q = q_out[0];

  // NOTE: For now, base movement will be ignored, set all to zero
  geometry_msgs::Pose2D base_pose;
  base_pose.x = 0;
  base_pose.y = 0;
  base_pose.theta = 0;

  std::vector<float> angles(5);
  for (int i = 0; i++; i<5) {
    angles.push_back(q(i));
  }

  jarvis_grasp::goal_pose msg;
  msg.base_pose = base_pose;
  msg.joint_angles = angles;
  pose_pub.publish(msg);
}

/* Extract kinematic data from urdf file for the youbot
 */
bool PoseGenerator::extractKinematicData(const urdf::Model & robot_model,
                                         const std::string base_frame,
                                         const std::string tip_frame,
                                         std::vector<double>& lower_limits,
                                         std::vector<double>& upper_limits) {
  boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(
                                                                             robot_model.getLink(tip_frame));

  while ((link) && (link->name != base_frame)) {
    boost::shared_ptr<urdf::Joint> joint = link->parent_joint;

    // Don't consider invalid, unknown or fixed joints
    if ((!joint) || (joint->type == urdf::Joint::UNKNOWN)
        || (joint->type == urdf::Joint::FIXED)) {
      // Continue with the next link in the kinematic chain
      link = link->getParent();

      continue;
    }

    // Extract the joint limits
    if (joint->type != urdf::Joint::CONTINUOUS) {
      if (joint->safety) {
        lower_limits.push_back(joint->safety->soft_lower_limit);
        upper_limits.push_back(joint->safety->soft_upper_limit);
      } else {
        lower_limits.push_back(joint->limits->lower);
        upper_limits.push_back(joint->limits->upper);
      }
    } else {
      lower_limits.push_back(-M_PI);
      upper_limits.push_back( M_PI);
    }

    // Continue with the next link in the kinematic chain
    link = link->getParent();
  }

  // The kinematic chain ended and the base frame was not found
  if (!link) return false;

  // The data has been extracted from the tip to the base, but it is required
  // the other way round
  std::reverse(lower_limits.begin(), lower_limits.end());
  std::reverse(upper_limits.begin(), upper_limits.end());

  return true;
}

/* Creates the IK object by loading kinematic data from urdf file.
 */
void PoseGenerator::loadIK() {

  // Get file path for urdf. Should be done with parameter server
  std::string urdf_file = ros::package::getPath("jarvis_grasp");
  urdf_file = urdf_file + "/urdf/youbot.urdf";

  urdf::Model robot_model;
  if (!robot_model.initFile(urdf_file)) {
    ROS_ERROR("Failed to parse urdf file");
  }
  ROS_INFO("Successfully parsed urdf file");

  // Set names for base and tip frame. Should be done with parameter
  // server
  std::string base_frame = "base_footprint";
  std::string tip_frame = "gripper_palm_link";

  std::vector<double> min_angle;
  std::vector<double> max_angle;

  if (!extractKinematicData(robot_model, base_frame, tip_frame, min_angle, max_angle)) {
    ROS_ERROR("Failed to extract kinematic data");
  }
  ROS_INFO("Successfully extracted kinematic data");

  ik = new youbot_arm_kinematics::InverseKinematics(min_angle, max_angle);
}
