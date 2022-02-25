#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

# Program parameter:
rospy.init_node('echo', anonymous=True)
topic= 'tic'

# ROS callback:
def echo(mesage):
    rospy.loginfo( mesage.data )

# ROS configuration:
rospy.Subscriber(topic, String, echo)

# run ROS node:
rospy.spin()