# Node for controlling bot with keyboard 
#!/usr/bin/env python3
import keyboard
import signal
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
settings = termios.tcgetattr(sys.stdin)
rospy.init_node("keyboard_control")
# creating Twist message object
vel = Twist()
vel.linear.x = 0
vel.angular.z = 0
pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
# function that returns the key pressed in keyboard
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
while True:
    key = getKey()
    # setting the linear and angular velocities of bot
    # based on the key pressed
    if key=="w":
        vel.linear.x = 0.1
        vel.angular.z =0
    if key=="s":
        vel.linear.x=-0.1
        vel.angular.z =0
    if key=="a":
        vel.angular.z =1.1
        vel.linear.x=0
    if key=="d":
        vel.angular.z=-1.1
        vel.linear.x=0
    if key=="x":
        vel.angular.z = 0
        vel.linear.x = 0

    rospy.loginfo(vel)
    pub.publish(vel)
