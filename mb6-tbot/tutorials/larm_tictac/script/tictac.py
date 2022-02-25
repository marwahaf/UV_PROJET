#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from larm_tictac import srv

# Program parameter:
rospy.init_node('tictac', anonymous=True)
pub = rospy.Publisher('tic', String, queue_size=10)
rate = rospy.Rate(1) # 10hz
msgs= ['tic', 'tac']

# ROS callback
def rate_service( data ):
    global rate
    rate= rospy.Rate( data.Value )
    return 1

# ROS node configuration
rospy.Service('rate', srv.Rate, rate_service)

# Program loop:
i= 0
while not rospy.is_shutdown():
    pub.publish(msgs[i])
    i= (i+1)%len(msgs)
    rate.sleep()