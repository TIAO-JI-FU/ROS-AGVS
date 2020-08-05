#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, termios, tty
import math

msg = """
Control agvs!
---------------------------
Moving around:
        w
   a    s    d
        x     

space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'w':(1,0),
        'a':(0,1),
        'd':(0,-1),
        'x':(-1,0)
           }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 1.5
turn = 0.1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('agvs_teleop')
    pub = rospy.Publisher('/agvs_wheel/cmd_vel', Twist, queue_size=5)
    front_right_motor_wheel = rospy.Publisher('/agvs/front_right_motor_wheel_joint_controller/command', Float64, queue_size=5)
    front_left_motor_wheel  = rospy.Publisher('/agvs/front_left_motor_wheel_joint_controller/command' , Float64, queue_size=5)
    back_right_motor_wheel  = rospy.Publisher('/agvs/back_right_motor_wheel_joint_controller/command' , Float64, queue_size=5)
    back_left_motor_wheel   = rospy.Publisher('/agvs/back_left_motor_wheel_joint_controller/command'  , Float64, queue_size=5)

    x = 0
    th = 0
    now_th = 0
    rotate_rate = math.pi/40
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            # Direction
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                if(moveBindings[key][1]== -1):
                    th = math.pi/2
                elif(moveBindings[key][1]== 1):
                    th = 0.0  
                count = 0

            # Stop
            elif key == ' ' or key == 's' :
                x = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed
            
            r = rospy.Rate(2)
            while(abs(th-now_th)>0.003):
                now_th = now_th + ((th-now_th)/abs(th-now_th))*rotate_rate
                front_right_motor_wheel.publish(now_th)
                front_left_motor_wheel.publish(now_th)
                back_right_motor_wheel.publish(now_th)
                back_left_motor_wheel.publish(now_th)
                r.sleep()
            
            twist = Twist()
            twist.linear.x = control_speed 
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0 
            twist.angular.y = 0 
            twist.angular.z = 0
            twist_motor = Twist()
            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        pub_motor.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
