#This message is a robot and object-agnostic representation of a pinching grasp

#The pose of the gripper, pre grasp- the origin is the point equally between the grippers,
#the faces of the grippers close along the x axis and the z axis points towards the wrist
geometry_msgs/Point point
geometry_msgs/Quaternion orientation 
float64 weight #probability that the grasp is good. Should be between zero and one.
