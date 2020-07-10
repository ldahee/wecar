#!/usr/bin/env python

import rospy
from topic_tutorial_2.msg import MyMsgsArr

name_topic = '/msgs_talk'
name_node = "pub_node_1"
 
if __name__ == '__main__':
    
    rospy.init_node(name_node, anonymous=True)
    
    pub = rospy.Publisher(name_topic, MyMsgsArr, queue_size=30)

    rate = rospy.Rate(10) # 10hz
    
    msgs_pub = MyMsgsArr()

    while not rospy.is_shutdown():

        msgs_pub.x = [1,2,3]
        msgs_pub.y = [4,5,6]

        pub.publish(msgs_pub)

        rate.sleep()

