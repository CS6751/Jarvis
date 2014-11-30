#include "ros/ros.h"
#include "ros/package.h"
#include <vector>
#include <urdf/model.h>
#include "youbot_arm_kinematics/inverse_kinematics.h"
#include "geometry_msgs/Pose2D.h"
#include "jarvis_grasp/goal_pose.h"
#include "jarvis_perception/GraspArray.h"
#include "jarvis_perception/GraspBox.h"


youbot_arm_kinematics::InverseKinematics* loadIK();

int main(int argc, char **argv) {
  ros::init(argc, argv, "ik_tester");
  ros::NodeHandle n;

  // Load kinematic data from URDF and use to create ik object
  youbot_arm_kinematics::InverseKinematics* ik = loadIK();


  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete ik;

  return 0;
}

bool extractKinematicData(const urdf::Model & robot_model,
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


youbot_arm_kinematics::InverseKinematics* loadIK() {

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

  return new youbot_arm_kinematics::InverseKinematics(min_angle, max_angle);
}
