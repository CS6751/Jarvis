#!/usr/bin/env python
# make prediction of human intent depending on the motor data

import rospy

def callback(data)
    rospy.loginfo(rospy.get_caller_id()+data.data)

def prediction():
    print '1'
    rospy.init_node('prediction', anonymous=True)

    rospy.Subscriber("/joint_states", sensor_msgs/JointState, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        prediction()
    except rospy.ROSInterruptException :pass



