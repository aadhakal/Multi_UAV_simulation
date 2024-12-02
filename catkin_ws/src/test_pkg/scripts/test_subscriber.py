#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(message):
    string_data = message.data
    print(string_data)

def subscriber():
    rospy.init_node('testNode2', anonymous=False)
    rospy.Subscriber('testTopic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()