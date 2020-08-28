#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

class agvs_parameter:
    def __init__(self, x_position, y_postition, angular):
        
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
        self.moving = False

        self.feedback_enable = False
        self.twist_wheel = Twist()
        self.twist_wheel.linear.x = 0.0
        self.twist_wheel.linear.y = 0.0
        self.twist_wheel.linear.z = 0.0
        self.twist_wheel.angular.x = 0.0
        self.twist_wheel.angular.y = 0.0
        self.twist_wheel.angular.z = 0.0

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
        
        # Set Serivce
        self.clear_ser = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

#   Get return of clear service.
    def Clear_ser_callback(self, data):
        pass

#   Navigation will send cmd_vel to topic, and we change it to our velocity, and send to agv.
    def Cmd_vel_callback(self, data):

        absolute_x = abs(self.goal_x_position - self.now_x_position)
        absolute_y = abs(self.goal_y_position - self.now_y_position)
        judge_x = self.Calculate_positive_or_negative(self.goal_x_position - self.now_x_position)
        judge_y = self.Calculate_positive_or_negative(self.goal_y_position - self.now_y_position)
        # Set velocity of wheels
        self.twist_wheel.linear.x = 0.0
        if(self.feedback_enable):
            speed = 0.0
            if((data.linear.x == 0.0) and (data.linear.y == 0.0)):
                if((absolute_x > 0.04) or (absolute_y > 0.04)):
                    if(self.first_times_zero):
                        push_data = GoalID()
                        self.move_base_cancel_pub.publish(push_data)
                    else:
                        self.first_times_zero = True
                else:
                    pass
            else:
                direction = judge_y if(self.rotate_state) else judge_x
                speed = (data.linear.x**2+data.linear.y**2)**0.5*direction
            # Set speed
            self.twist_wheel.linear.x = speed
            self.moving = True
            self.feedback_enable = False
            self.wheel_pub.publish(self.twist_wheel)
            self.Push_rotate(self.now_angular) 
            self.moving = False
            self.feedback_enable = True
        else:
            pass

#   If you send a goal to the topic, here will get it and set the place of goal in navigation.
    def Goal_callback(self, data):
        self.clear_ser()

        self.goal_x_position = data.goal.target_pose.pose.position.x
        self.goal_y_position = data.goal.target_pose.pose.position.y

        absolute_x = abs(self.goal_x_position - self.now_x_position)
        absolute_y = abs(self.goal_y_position - self.now_y_position)

        if(absolute_y >= absolute_x):
            self.rotate_state = True
            self.rotate_angular = math.pi/2
        else:
            self.rotate_state = False
            self.rotate_angular = 0.00000
        
        self.feedback_enable = False
        r = rospy.Rate(400)
        while(abs(self.rotate_angular - self.now_angular) > 0.001):
            judge_angular = self.Calculate_positive_or_negative(self.rotate_angular - self.now_angular)
            self.now_angular = self.now_angular + (judge_angular * self.rotate_rate)
            self.Push_rotate(self.now_angular)
            self.twist_wheel.linear.x = 0.0
            self.wheel_pub.publish(self.twist_wheel)
            r.sleep()
        rospy.sleep(3)
        self.feedback_enable = True
        self.first_times_zero = False

#   Get now place of AMCL in the global map.
    def Feedback_callback(self, data):
        if(~self.moving):
            self.now_x_position = data.pose.pose.position.x
            self.now_y_position = data.pose.pose.position.y
            # print("Now_x",self.now_x_position)
            # print("Now_y",self.now_y_position)
            absolute_x = abs(self.goal_x_position - self.now_x_position)
            absolute_y = abs(self.goal_y_position - self.now_y_position)
            judge_x = self.Calculate_positive_or_negative(self.goal_x_position - self.now_x_position)
            judge_y = self.Calculate_positive_or_negative(self.goal_y_position - self.now_y_position)
            p_or_n = judge_x * judge_y
            p_or_n = p_or_n if absolute_x > absolute_y else p_or_n*(-1)
            if(self.feedback_enable):
                way = absolute_y if(self.rotate_state) else absolute_x
                self.now_angular = math.acos(way/((absolute_x**2+absolute_y**2)**0.5))*p_or_n*0.9 + self.rotate_angular
            else:
                pass
        else:
            pass

#   Push angle of wheel.
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