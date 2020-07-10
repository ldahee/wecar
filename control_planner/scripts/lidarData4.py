#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos, sin, pi
from geometry_msgs.msg import Point32

class simple_controller:
    def __init__(self):
        rospy.init_node("simple_controller", anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position', Float64, queue_size=1)
        self.pcd_pub = rospy.Publisher('laser2pcd', PointCloud, queue_size=1)
        
        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self, msg):
        pcd=PointCloud()
        motor_msg, servo_msg = Float64(), Float64()
        pcd.header.frame_id=msg.header.frame_id
        angle=0
        rpm_gain = 4614
        servo_gain = -1.2135
        servo_offset = 0.5304

        for r in msg.ranges :
            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)
            angle=angle+(1.0/180*pi)
            # print(angle)
            if r<12:
                pcd.points.append(tmp_point)

        count, left_count, right_count = 0, 0, 0
        for point in pcd.points:
            if point.x > 0 and point.x < 1 and point.y > -1 and point.y < 2 :
                count=count+1

            if point.x > -2 and point.x < 0 and point.y > -1 and point.y < 2:
                left_count += 1

            if point.x > 1 and point.x < 3 and point.y > -1 and point.y < 2:
                right_count += 1 

        print("count:", count)
        print("left:", left_count)
        print("right:", right_count)
        print("--------------------------------")
            
        if count > 40:
            servo_msg = servo_gain * (0 /180*pi) + servo_offset
            if (right_count < 10 and left_count > 40) or right_count < left_count:
                servo_msg = servo_gain * (90 /180*pi) + servo_offset
                motor_msg.data=4000

            if (right_count > 40 and left_count < 10) or right_count > left_count:
                servo_msg = servo_gain * (270 /180*pi) + servo_offset
                motor_msg.data=4000

            # if right_count == 0 and left_count == 0:
            #     servo_msg = servo_gain * (270 /180*pi) + servo_offset
            #     motor_msg.data=4000

        else:
            motor_msg.data=4000
            servo_msg = servo_gain * (0 /180*pi) + servo_offset

            if 20 < left_count:
                servo_msg = servo_gain * (90 /180*pi) + servo_offset

            if right_count > 20 :
                servo_msg = servo_gain * (270 /180*pi) + servo_offset
        
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)
        self.pcd_pub.publish(pcd)

if __name__ == '__main__':
    try:
        test_track = simple_controller()
    except rospy.ROSInterruptException:
        pass