#ifndef POSE_GENERATOR
#define POSE_GENERATOR

#include "ros/ros.h"
#include "jarvis_perception/GraspArray.h"
#include "youbot_arm_kinematics/inverse_kinematics.h"
#include <urdf/model.h>
#include <vector>


class PoseGenerator {

 public:
  void callback(const jarvis_perception::GraspArray::ConstPtr&);

  PoseGenerator(ros::NodeHandle n);
  ~PoseGenerator();

 private:
  youbot_arm_kinematics::InverseKinematics* ik;

  ros::Publisher pose_pub;
  ros::Subscriber sub;

  void loadIK();
  bool extractKinematicData(const urdf::Model & robot_model,
                            const std::string base_frame,
                            const std::string tip_frame,
                            std::vector<double>& lower_limits,
                            std::vector<double>& upper_limits);

};

#endif
