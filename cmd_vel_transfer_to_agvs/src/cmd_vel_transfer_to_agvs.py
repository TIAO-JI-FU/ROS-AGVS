#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseActionGoal

class agvs_parameter:
    def __init__(self, x_position, y_postition, angular):

        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist , self.cmd_vel_callback)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal , self.goal_callback)
        self.feedback_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.feedback_callback)
        
        self.pub_wheel = rospy.Publisher('/agvs_wheel/cmd_vel', Twist, queue_size=5)
        self.front_right_motor_wheel = rospy.Publisher('/agvs/front_right_motor_wheel_joint_controller/command', Float64, queue_size=5)
        self.front_left_motor_wheel  = rospy.Publisher('/agvs/front_left_motor_wheel_joint_controller/command' , Float64, queue_size=5)
        self.back_right_motor_wheel  = rospy.Publisher('/agvs/back_right_motor_wheel_joint_controller/command' , Float64, queue_size=5)
        self.back_left_motor_wheel   = rospy.Publisher('/agvs/back_left_motor_wheel_joint_controller/command'  , Float64, queue_size=5)
        
        self.rotate_angular = angular
        self.rotate_rate = math.pi/4000
        self.now_angular = angular
        self.now_x_position = x_position
        self.now_y_position = y_postition
        self.goal_x_position = x_position
        self.goal_y_position = y_postition
        self.feedback_enable = False

    def cmd_vel_callback(self, data):

        absolute_x = abs(self.goal_x_position - self.now_x_position)
        absolute_y = abs(self.goal_y_position - self.now_y_position)
        judge_x = self.calculate_positive_or_negative(self.goal_x_position - self.now_x_position)
        judge_y = self.calculate_positive_or_negative(self.goal_y_position - self.now_y_position)
        
        change = 1.0
        if(absolute_x > 0.03):
            pass
        elif(judge_y == -1):
            change = -1.0
        else:
            pass
        
        twist_wheel = Twist()
        twist_wheel.linear.x = data.linear.x * change
        twist_wheel.linear.y = 0
        twist_wheel.linear.z = 0
        twist_wheel.angular.x = 0
        twist_wheel.angular.y = 0
        twist_wheel.angular.z = 0

        if(self.feedback_enable):
            self.pub_wheel.publish(twist_wheel)
            self.feedback_enable = False
            self.front_right_motor_wheel.publish(self.now_angular)
            self.front_left_motor_wheel.publish(self.now_angular)
            self.back_right_motor_wheel.publish(self.now_angular)
            self.back_left_motor_wheel.publish(self.now_angular)
            self.feedback_enable = True

    def goal_callback(self, data):

        self.goal_x_position = data.goal.target_pose.pose.position.x
        self.goal_y_position = data.goal.target_pose.pose.position.y

        absolute_x = abs(self.goal_x_position - self.now_x_position)
        absolute_y = abs(self.goal_y_position - self.now_y_position)

        if((absolute_x < 0.1) and (absolute_y > 0.1)):
            self.rotate_angular = -(math.pi/2)
        elif((absolute_y < 0.1) and (absolute_x > 0.1)):
            self.rotate_angular = 0.00000
        else:
            pass

        self.feedback_enable = False
        r = rospy.Rate(500)
        while(abs(self.rotate_angular - self.now_angular) > 0.001):
            judge_angular = self.calculate_positive_or_negative(self.rotate_angular - self.now_angular)
            self.now_angular = self.now_angular + (judge_angular * self.rotate_rate)
            self.front_right_motor_wheel.publish(self.now_angular)
            self.front_left_motor_wheel.publish(self.now_angular)
            self.back_right_motor_wheel.publish(self.now_angular)
            self.back_left_motor_wheel.publish(self.now_angular)
            r.sleep()
        self.feedback_enable = True

    def feedback_callback(self, data):

        self.now_x_position = data.pose.pose.position.x
        self.now_y_position = data.pose.pose.position.y
        absolute_x = abs(self.goal_x_position - self.now_x_position)
        absolute_y = abs(self.goal_y_position - self.now_y_position)
        judge_x = self.calculate_positive_or_negative(self.goal_x_position - self.now_x_position)
        judge_y = self.calculate_positive_or_negative(self.goal_y_position - self.now_y_position)
        
        if((absolute_x > 0.03) or (absolute_y > 0.03)):
            p_or_n = judge_x * judge_y
            p_or_n = p_or_n if absolute_x > absolute_y else p_or_n*(-1)
            if(self.feedback_enable):
                way = max(absolute_x,absolute_y)
                self.now_angular = math.acos(way/((absolute_x**2+absolute_y**2)**0.5))*p_or_n + self.rotate_angular
    
    def calculate_positive_or_negative(self, data):

        return (data)/abs(data)
        
if __name__=="__main__":
    rospy.init_node('cmd_vel_transfer_to_agvs')
    initial_x = 2.22
    initial_y = 4.675
    initial_angular = 0.0
    agvs = agvs_parameter(initial_x,initial_y,initial_angular)
    rospy.spin()