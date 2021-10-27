#!/usr/bin/env python  
import rospy
import numpy as np
import math
import tf2_ros
from bot_msgs.srv import Inverse_kinematics,Inverse_kinematicsResponse
from InverseKinematics import Ik

def Joint_control(jnt):
    rospy.wait_for_service('pose_to_arm_ontrol')
    try:
        joint_req = rospy.ServiceProxy('pose_to_arm_ontrol', Inverse_kinematics)
        resp = joint_req(jnt[0],jnt[1],jnt[2],jnt[3])
        return resp.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    rospy.init_node('pose_control_client')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    i = 0
    delta  = np.array([0,0,0,0])
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("s1", 'object', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        #msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        current =  np.array([trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z])
        if i!=0:
            delta = (current - prev)
        if i == 0:
            delta = np.array([0,0,0,0])
        if i == 20 :
            x = input("object is detected and its velocity is zero please type 1 to continue")
            print(trans)
            if int(x):
                for j in range(91):
                    joints =  Ik(trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z,0)
                    if not np.isnan(joints).any():
                            break
                Joint_control(joints)
                print(joints)
        prev = current
        i+=1