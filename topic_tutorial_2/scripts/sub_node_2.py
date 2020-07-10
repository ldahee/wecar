#!/usr/bin/env python

import rospy
from topic_tutorial_2.msg import MyMsgsArr

name_topic = '/msgs_talk'
name_node = 'sub_node'

def callback(msgs):

    rospy.loginfo(msgs.x[1]**2 - msgs.y[1])
 
if __name__ == '__main__':

    rospy.init_node(name_node, anonymous=True)
    
    sub = rospy.Subscriber(name_topic, MyMsgsArr, callback)

    rospy.spin()
    