#!/usr/bin/env python
import rospy
import numpy as np
import std_msgs.msg
from movimiento_puzzlebot.msg import set_point
from std_msgs.msg import String
from std_msgs.msg import String

if __name__=='__main__':
    rospy.init_node("path_generator")
    tv=rospy.get_param('tv')
    ta= rospy.get_param('ta')
    help_msg = String()
    help_msg2 = String()
    help_msg.data = tv
    help_msg2.data = ta
    pub = rospy.Publisher('/help_msg', String, queue_size = 1)
    pub2 = rospy.Publisher('/help_msg2', String, queue_size = 1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
	pub.publish(help_msg)
        pub2.publish(help_msg2)
        print(help_msg)
        print(help_msg2)
        rate.sleep()

	

		
