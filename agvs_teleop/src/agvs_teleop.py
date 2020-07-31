#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

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
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    pub_motor = rospy.Publisher('/agvs_motor_wheel/cmd_vel', Twist, queue_size=5)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
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
                th = moveBindings[key][1]
                count = 0

            # Stop
            elif key == ' ' or key == 's' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed 
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0 
            twist.angular.y = 0 
            twist.angular.z = 0
            twist_motor = Twist()
            twist_motor.linear.x = control_turn 
            twist_motor.linear.y = 0
            twist_motor.linear.z = 0
            twist_motor.angular.x = 0 
            twist_motor.angular.y = 0 
            twist_motor.angular.z = 0
            pub.publish(twist)
            pub_motor.publish(twist_motor)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        pub_motor.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
