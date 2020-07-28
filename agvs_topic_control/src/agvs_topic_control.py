#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64,Bool
linear = 0
angular = 0
def callback_linear(data):
    global linear
    linear = data.data
def callback_angular(data):
    global angular
    if(data.data):
        angular = 1.57
    else:
        angular = 0

def listen_my_topic():
    rospy.init_node('listen_my_topic', anonymous=True)
    pub_fr_rotate = rospy.Publisher('agvs/front_right_motor_wheel_joint_controller/command', Float64, queue_size=10)
    pub_fl_rotate = rospy.Publisher('agvs/front_left_motor_wheel_joint_controller/command', Float64, queue_size=10)
    pub_br_rotate = rospy.Publisher('agvs/back_right_motor_wheel_joint_controller/command', Float64, queue_size=10)
    pub_bl_rotate = rospy.Publisher('agvs/back_left_motor_wheel_joint_controller/command', Float64, queue_size=10)
    
    pub_fr_move = rospy.Publisher('agvs/front_right_wheel_joint_controller/command', Float64, queue_size=10)
    pub_fl_move = rospy.Publisher('agvs/front_left_wheel_joint_controller/command', Float64, queue_size=10)
    pub_br_move = rospy.Publisher('agvs/back_right_wheel_joint_controller/command', Float64, queue_size=10)
    pub_bl_move = rospy.Publisher('agvs/back_left_wheel_joint_controller/command', Float64, queue_size=10)

    rospy.Subscriber("my_topic_distance", Float64, callback_linear)
    rospy.Subscriber("my_topic_angle", Bool, callback_angular)
    
    rate = rospy.Rate(5)
    now_speed = 0
    now_rotate = 0
    rate_speed = 0.2
    rate_rotate = 0.157
    while not rospy.is_shutdown():
        global linear
        global angular
        if(now_speed == linear):
            #linear = 0
            pass
        elif(linear > now_speed):
            now_speed = now_speed + rate_speed
        else:
            now_speed = now_speed - rate_speed

        if(angular == now_rotate):
            pass
        elif(angular > now_rotate):
            now_rotate = now_rotate + rate_rotate
        else:
            now_rotate = now_rotate -rate_rotate
        
        pub_fr_rotate.publish(float(now_rotate))
        pub_fl_rotate.publish(float(now_rotate))
        pub_br_rotate.publish(float(now_rotate))
        pub_bl_rotate.publish(float(now_rotate))
        pub_fr_move.publish(float(now_speed))
        pub_fl_move.publish(float(now_speed))
        pub_br_move.publish(float(now_speed))
        pub_bl_move.publish(float(now_speed))
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        listen_my_topic()
    except rospy.ROSInterruptException:
        pass