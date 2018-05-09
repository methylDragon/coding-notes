#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    
def listener():
    rospy.init_node('listener', anonymous = True)
    
    # Callback function gets run each time a message is received
    rospy.Subscriber("chatter", String, callback)

    # spin() keeps Python from exiting until this node is stopped
    rospy.spin()

def sub_main():
    listener()
