#This message is a robot and object-agnostic representation of a pinching grasp

Header header
# Name of the grasp
string name

#The pose of the gripper, pre grasp- the origin is the point equally between the grippers,
#the faces of the grippers close along the x axis and the z axis points towards the wrist
geometry_msgs/Pose pose
