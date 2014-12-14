README for jarvis_perception

jarvis_perception provides visual data

topics
------
/objects/* (* is the name of various object - hand, finger, target)

nodes
-------
return_grips
object_publisher

service types
-------------
ReturnGrips
{
	returns GripArray
}

message types
------------
*AxisAlignedBox*: 
{
Header 				header
string 				name
geometry_msgs/Pose 	pose
float64 			w
float64 			l
float64 			h
}
*GripArray*
{
GraspBox[]			grasps
}
*GraspBox*
{
geometry_msgs/Point 		point
geometry_msgs/Quaternion 	orientation
float64 					weight	
}