#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

class agvs_parameter:
    def __init__(self, x_position, y_postition, angular):
        
        # Set Subscriber
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist , self.Cmd_vel_callback)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal , self.Goal_callback)
        self.feedback_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped , self.Feedback_callback)

        # Set Publisher
        self.wheel_pub = rospy.Publisher('/agvs_wheel/cmd_vel', Twist, queue_size=5)
        self.move_base_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=5)
        self.front_right_motor_wheel_pub = rospy.Publisher('/agvs/front_right_motor_wheel_joint_controller/command', Float64, queue_size=5)
        self.front_left_motor_wheel_pub  = rospy.Publisher('/agvs/front_left_motor_wheel_joint_controller/command' , Float64, queue_size=5)
        self.back_right_motor_wheel_pub  = rospy.Publisher('/agvs/back_right_motor_wheel_joint_controller/command' , Float64, queue_size=5)
        self.back_left_motor_wheel_pub   = rospy.Publisher('/agvs/back_left_motor_wheel_joint_controller/command'  , Float64, queue_size=5)
        
        # Set Parameters
        self.rotate_angular = angular
        self.rotate_rate = math.pi/4000
        self.rotate_state = False

        self.now_angular = angular
        self.now_x_position = x_position
        self.now_y_position = y_postition

        self.goal_x_position = x_position
        self.goal_y_position = y_postition

        self.first_times_zero = False

        self.feedback_enable = False
        self.twist_wheel = Twist()
        self.twist_wheel.linear.x = 0
        self.twist_wheel.linear.y = 0
        self.twist_wheel.linear.z = 0
        self.twist_wheel.angular.x = 0
        self.twist_wheel.angular.y = 0
        self.twist_wheel.angular.z = 0

    def Cmd_vel_callback(self, data):

        absolute_x = abs(self.goal_x_position - self.now_x_position)
        absolute_y = abs(self.goal_y_position - self.now_y_position)
        judge_x = self.Calculate_positive_or_negative(self.goal_x_position - self.now_x_position)
        judge_y = self.Calculate_positive_or_negative(self.goal_y_position - self.now_y_position)

        # Set velocity of wheels
        self.twist_wheel.linear.x = 0
        if(self.feedback_enable):
            self.twist_wheel.linear.x = data.linear.y if(self.rotate_state) else data.linear.x
            judge_dir = judge_y if(self.rotate_state) else judge_x
            if(absolute_x > 0.05 or absolute_y > 0.05):
                if((self.twist_wheel.linear.x * judge_dir <= 0) and self.first_times_zero):
                    push_data = GoalID()
                    self.move_base_cancel_pub.publish(push_data)
                else:
                    self.first_times_zero = True
            self.feedback_enable = False
            self.Push_rotate(self.now_angular)
            self.feedback_enable = True
        else:
            pass
        self.wheel_pub.publish(self.twist_wheel)

    def Goal_callback(self, data):

        self.goal_x_position = data.goal.target_pose.pose.position.x
        self.goal_y_position = data.goal.target_pose.pose.position.y

        absolute_x = abs(self.goal_x_position - self.now_x_position)
        absolute_y = abs(self.goal_y_position - self.now_y_position)

        if((absolute_x < 0.1) and (absolute_y > 0.1)):
            self.rotate_state = True
            self.rotate_angular = math.pi/2
        elif((absolute_y < 0.1) and (absolute_x > 0.1)):
            self.rotate_state = False
            self.rotate_angular = 0.00000
        else:
            rospy.signal_shutdown("Some problam of next point")

        self.feedback_enable = False
        r = rospy.Rate(500)
        while(abs(self.rotate_angular - self.now_angular) > 0.001):
            judge_angular = self.Calculate_positive_or_negative(self.rotate_angular - self.now_angular)
            self.now_angular = self.now_angular + (judge_angular * self.rotate_rate)
            self.Push_rotate(self.now_angular)

            self.twist_wheel.linear.x = 0.0
            self.wheel_pub.publish(self.twist_wheel)
            r.sleep()
        self.feedback_enable = True
        self.first_times_zero = False

    def Feedback_callback(self, data):

        self.now_x_position = data.pose.pose.position.x
        self.now_y_position = data.pose.pose.position.y
        absolute_x = abs(self.goal_x_position - self.now_x_position)
        absolute_y = abs(self.goal_y_position - self.now_y_position)
        judge_x = self.Calculate_positive_or_negative(self.goal_x_position - self.now_x_position)
        judge_y = self.Calculate_positive_or_negative(self.goal_y_position - self.now_y_position)
        
        if((absolute_x > 0.03) or (absolute_y > 0.03)):
            p_or_n = judge_x * judge_y
            p_or_n = p_or_n if absolute_x > absolute_y else p_or_n*(-1)
            if(self.feedback_enable):
                way = absolute_y if(self.rotate_state) else absolute_x
                self.now_angular = math.acos(way/((absolute_x**2+absolute_y**2)**0.5))*p_or_n*0.7 + self.rotate_angular
                #print("Now_angular: ",self.now_angular)
                #print("Now_X: ",self.now_x_position)
                #print("Now_Y: ",self.now_y_position)
            else:
                pass
        else:
            pass

    def Push_rotate(self, data):
        self.front_right_motor_wheel_pub.publish(data)
        self.front_left_motor_wheel_pub.publish(data)
        self.back_right_motor_wheel_pub.publish(data)
        self.back_left_motor_wheel_pub.publish(data)

    def Calculate_positive_or_negative(self, data):

        return (data)/abs(data)

if __name__=="__main__":
    rospy.init_node('cmd_vel_transfer_to_agvs')
    initial_x = 8.065
    initial_y = 4.675
    initial_angular = 0.0
    agvs = agvs_parameter(initial_x,initial_y,initial_angular)
    rospy.spin()