#ifndef POSE_GENERATOR
#define POSE_GENERATOR

#include "ros/ros.h"
#include "jarvis_perception/GraspArray.h"
#include "youbot_arm_kinematics/inverse_kinematics.h"
#include <urdf/model.h>
#include <vector>
#include <kdl/jntarray.hpp>
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>


class PoseGenerator {

 public:
  void loc_callback(const jarvis_perception::GraspArray::ConstPtr&);
  void config_callback(sensor_msgs::JointState);
  void marker_callback(visualization_msgs::Marker);

  PoseGenerator(ros::NodeHandle n);
  ~PoseGenerator();

 private:
  youbot_arm_kinematics::InverseKinematics* ik;
  KDL::JntArray cur_q; // Required for IK function, but not used!!
  sensor_msgs::JointState cur_state;
  visualization_msgs::Marker cur_pos; // Current position of human

  ros::Publisher pose_pub;
  ros::Subscriber loc_sub;
  ros::Subscriber config_sub;
  ros::Subscriber marker_sub;

  tf::TransformListener tf_listener;

  void calc_weights(std::vector<KDL::JntArray>&, float[], float[]);
  void loadIK();
  bool extractKinematicData(const urdf::Model & robot_model,
                            const std::string base_frame,
                            const std::string tip_frame,
                            std::vector<double>& lower_limits,
                            std::vector<double>& upper_limits);

};

#endif
