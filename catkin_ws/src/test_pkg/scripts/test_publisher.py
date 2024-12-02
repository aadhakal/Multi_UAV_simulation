#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('testTopic', String, queue_size=10)

    rospy.init_node('testNode', anonymous=False)

    rate = rospy.Rate(10) # in hz value, how often we want to subscribe our data

    while not rospy.is_shutdown():
        string_data = "Let's fly this!"
        pub.publish(string_data)
        rate.sleep()
    
if __name__=='__main__':
    publisher()