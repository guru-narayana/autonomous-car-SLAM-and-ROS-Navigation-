#! /usr/bin/python3

from RPi import GPIO
import signal
import rospy
import math
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

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

pub = rospy.Publisher("g_Pose",Pose,queue_size = 10)
rospy.init_node("pose_data")
p = Pose()
p.position.x = 0
p.position.y = 0
p.position.z = 0
p.orientation.x = 0
p.orientation.y = 0
p.orientation.z = 0
p.orientation.w = 0
previous_time_l = rospy.get_time()
previous_time_r = rospy.get_time()
left_dist = 0
right_dist = 0
left_dist_prev = 0
right_dist_prev = 0
dl = 0
dr = 0
theta = 0
clkLastState1 = GPIO.input(clk1)
clkLastState2 = GPIO.input(clk2)

try:

        while True:
                clkState1 = GPIO.input(clk1)
                clkState2 = GPIO.input(clk2)
                dtState1= GPIO.input(dt1)
                dtState2= GPIO.input(dt2)
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
                dl = left_dist - left_dist_prev
                dr = right_dist - right_dist_prev
                theta += (dr-dl)/0.19
                p.position.x += (dr+dl)*math.cos(theta)
                p.position.y += (dr+dl)*math.sin(theta)
                rot = Rotation.from_euler('xyz', [0, 0,theta ], degrees=False)
                rot_quat = rot.as_quat()
                p.orientation.z = rot_quat[2]
                p.orientation.w = rot_quat[3]
                rospy.loginfo(p)
                pub.publish(p)
                left_dist_prev = left_dist
                right_dist_prev = right_dist
                clkLastState1 = clkState1
                clkLastState2 = clkState2
                signal.signal(signal.SIGINT, keyboardInterruptHandler)
                
finally:
        GPIO.cleanup()
