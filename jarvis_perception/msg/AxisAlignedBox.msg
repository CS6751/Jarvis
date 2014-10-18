#The pose and size of a 3d box. center is at 0,0,0 in body coordinates
# corners at +/- w/2 in x_body, +/- l/2 in y_body, +/- h/2 in z_body
Header header
#Name or ID
string name
#Pose in the global frame
geometry_msgs/Pose pose
float64 w
float64 l
float64 h

