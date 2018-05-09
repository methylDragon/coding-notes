#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rawr_package import rawr

def talker():
    # Create a publisher object
    # It publishes String messages to the topic 'chatter'
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    
    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        rawr_str = rawr()
        rospy.loginfo(rawr_str)
        
        pub.publish(rawr_str)
        rate.sleep()

def pub_main():
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
