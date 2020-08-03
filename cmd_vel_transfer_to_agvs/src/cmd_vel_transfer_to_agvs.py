#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import termios, sys

def callback(data):
    print(data.linear.x)
    print(data.linear.y)
    print(data.linear.z)
    print(data.angular.x)
    print(data.angular.y)
    print(data.angular.z)
    print("-------------------------")
    pub_wheel = rospy.Publisher('/agvs_wheel/cmd_vel', Twist, queue_size=5)
    pub_motor = rospy.Publisher('/agvs_motor_wheel/cmd_vel', Twist, queue_size=5)
    twist_wheel = Twist()
    twist_wheel.linear.x = data.linear.x 
    twist_wheel.linear.y = 0
    twist_wheel.linear.z = 0
    twist_wheel.angular.x = 0 
    twist_wheel.angular.y = 0 
    twist_wheel.angular.z = 0
    twist_motor = Twist()
    twist_motor.linear.x = data.angular.z 
    twist_motor.linear.y = 0
    twist_motor.linear.z = 0
    twist_motor.angular.x = 0 
    twist_motor.angular.y = 0 
    twist_motor.angular.z = 0
    pub_wheel.publish(twist_wheel)
    pub_motor.publish(twist_motor)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('cmd_vel_transfer_to_agvs')
    sub_move = rospy.Subscriber('/cmd_vel', Twist , callback)
    rospy.spin()