#! /usr/bin/python

from RPi import GPIO
import signal
import rospy
from geometry_msgs.msg import Twist

clk1 = 13 #left wheel
dt1 = 6
clk2 = 19 #right Wheel
dt2 = 26

def keyboardInterruptHandler(signal, frame):
    print("this is velocity publisher signing off ...")
    exit(0)

GPIO.setmode(GPIO.BCM)
GPIO.setup(clk1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(clk2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

pub = rospy.Publisher("g_cmd_vel",Twist,queue_size = 10)
rospy.init_node("encoders_data")

vel = Twist()
vel.linear.x = 0
vel.linear.y = 0
vel.linear.z = 0
vel.angular.x = 0
vel.angular.y = 0 
vel.angular.z = 0

previous_time_l = rospy.get_time()
previous_time_r = rospy.get_time()

left_vel = 0
right_vel = 0
left_dist = 0
right_dist = 0
dl = 0
dr = 0
left_dist_prev = 0
right_dist_prev = 0
clkLastState1 = GPIO.input(clk1)
clkLastState2 = GPIO.input(clk2)

try:

        while True:
                clkState1 = GPIO.input(clk1)
                clkState2 = GPIO.input(clk2)
                dtState1= GPIO.input(dt1)
                dtState2= GPIO.input(dt2)
                present_time = rospy.get_time()
                dtl = (present_time - previous_time_l)
                dtr = (present_time - previous_time_r)
                if clkState1 != clkLastState1:
                        if dtState1 != clkState1:
                                left_dist+= 0.0055   
                        else:
                                left_dist+= -0.0055
                if clkState2 != clkLastState2:
                        if dtState2 != clkState2:
                                right_dist+= 0.0055
                        else:
                                right_dist+= -0.0055
		dl = left_dist-left_dist_prev
		dr = right_dist-right_dist_prev
                if abs(dtl) > 0.1 and abs(dtr) > 0.1:
                    left_vel = (left_dist-left_dist_prev)/dtl
                    left_dist_prev = left_dist
                    previous_time_l = present_time
                    right_vel = (right_dist-right_dist_prev)/dtr
                    right_dist_prev = right_dist
                    previous_time_r = present_time
                clkLastState1 = clkState1
                clkLastState2 = clkState2
                vel.linear.x = (left_vel + right_vel)/2
                vel.angular.z = (right_vel - left_vel)/(0.195)
                vel.linear.x  = round(vel.linear.x ,2)
                vel.angular.z = round(vel.angular.z,2)
                pub.publish(vel)
                rospy.loginfo(vel)
                signal.signal(signal.SIGINT, keyboardInterruptHandler)
                
finally:
        GPIO.cleanup()
