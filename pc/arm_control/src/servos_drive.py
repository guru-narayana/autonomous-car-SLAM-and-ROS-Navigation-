#!/usr/bin/env python
import rospy
import time
from bot_msgs.msg import servo
from sensor_msgs.msg import JointState

def s3(val):
    val = (val*591) + 1471
    val = max(544,min(val,2400))
    return val
def s1(val):
    val = (val*591) + 1471
    val = max(544,min(val,2400))
    return val
def s2(val):
    val = (val*(442.3)) + 1923
    val = max(950,min(val,2100))
    return val
def s4(val):
    val = (val*(-591)) + 1471
    val = max(544,min(val,2400))
    return val

servo_pos = servo()
pub = rospy.Publisher('servo_pos', servo, queue_size=1)
def callback(js):
    servo_pos.s1 = s1(js.position[0])
    servo_pos.s2 = s2(js.position[1])
    servo_pos.s3 = s3(js.position[2])
    servo_pos.s4 = s4(js.position[3])
    try:
        servo_pos.s5 = js.position[4]
    except:
        servo_pos.s5 = 2000
    pub.publish(servo_pos)
rospy.init_node('servo_drive', anonymous=True)
rospy.Subscriber("robot_simulation/joint_states", JointState, callback)
rospy.spin()