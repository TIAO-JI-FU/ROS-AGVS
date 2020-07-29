#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64,Bool

start = False
now_speed = 0.0
now_rotate = 0.0
job = [-3200,-2400,-1000,1600,3700,-850,-1000,1600]

def callback_start(data):
    global start
    start = True

def push_my_data():
    pub_fr_rotate = rospy.Publisher('agvs/front_right_motor_wheel_joint_controller/command', Float64, queue_size=10)
    pub_fl_rotate = rospy.Publisher('agvs/front_left_motor_wheel_joint_controller/command', Float64, queue_size=10)
    pub_br_rotate = rospy.Publisher('agvs/back_right_motor_wheel_joint_controller/command', Float64, queue_size=10)
    pub_bl_rotate = rospy.Publisher('agvs/back_left_motor_wheel_joint_controller/command', Float64, queue_size=10)
    
    pub_fr_move = rospy.Publisher('agvs/front_right_wheel_joint_controller/command', Float64, queue_size=10)
    pub_fl_move = rospy.Publisher('agvs/front_left_wheel_joint_controller/command', Float64, queue_size=10)
    pub_br_move = rospy.Publisher('agvs/back_right_wheel_joint_controller/command', Float64, queue_size=10)
    pub_bl_move = rospy.Publisher('agvs/back_left_wheel_joint_controller/command', Float64, queue_size=10)

    global now_rotate
    pub_fr_rotate.publish(float(now_rotate))
    pub_fl_rotate.publish(float(now_rotate))
    pub_br_rotate.publish(float(now_rotate))
    pub_bl_rotate.publish(float(now_rotate))

    global now_speed
    pub_fr_move.publish(float(now_speed))
    pub_fl_move.publish(float(now_speed))
    pub_br_move.publish(float(now_speed))
    pub_bl_move.publish(float(now_speed))

def listen_my_topic():
    rospy.init_node('listen_my_topic', anonymous=True)
    rospy.Subscriber("callback_start", Bool, callback_start)
    
    rate = rospy.Rate(100)
    
    global now_speed
    max_speed = 0.0
    rate_speed = 0.05
    
    global now_rotate
    max_rotate = 0.0
    rate_rotate = math.pi/800

    now_count_time = 0
    max_count_time = 0

    now_job_count = -1
    next_job_count = 0

    ang_or_spe = False
    while not rospy.is_shutdown():
        if(start):
            if(now_job_count != next_job_count):
                if(next_job_count >= len(job)):
                    return 0
                now_job_count = next_job_count
                max_count_time = abs(job[now_job_count])
                max_speed = (job[now_job_count]/abs(job[now_job_count]))*10.0
                max_rotate = (now_job_count%2)*(math.pi/2)
            if(now_count_time == max_count_time):
                max_speed = 0.0

            if(ang_or_spe == False):
                if(abs(max_rotate - now_rotate)<0.00001):
                    ang_or_spe = True
                    now_count_time = 0
                elif(max_rotate > now_rotate):
                    now_rotate = now_rotate + rate_rotate
                    push_my_data()
                else:
                    now_rotate = now_rotate -rate_rotate
                    push_my_data()
            else:
                now_count_time+=1
                if(abs(max_speed - now_speed)<0.00001):
                    pass
                elif(max_speed > now_speed):
                    now_speed = now_speed + rate_speed
                    push_my_data()
                else:
                    now_speed = now_speed - rate_speed
                    push_my_data()

            if((max_speed == 0.0) and (abs(max_speed - now_speed)<0.00001)):
                ang_or_spe = False
                rospy.sleep(2.0)
                next_job_count+=1

        print("-------------------------")
        print("max_rotate: ",max_rotate)
        print("now_rotate: ",now_rotate)
        print("now_speed: ",now_speed)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        listen_my_topic()
    except rospy.ROSInterruptException:
        pass