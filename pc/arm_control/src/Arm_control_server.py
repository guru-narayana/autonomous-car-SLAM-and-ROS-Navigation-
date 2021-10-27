#! /usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import JointState
from bot_msgs.msg import joint_controlAction, joint_controlGoal, joint_controlResult, joint_controlFeedback
import time
pub = rospy.Publisher('robot_simulation/joint_states', JointState, queue_size=1)
jnt = JointState()
begin = 1
jnt.name = ["j1","j2","j3","j4","j5"]
def eq_params(vel,theta_final,theta_initial):
    tf = abs(theta_final-theta_initial)/vel
    if theta_final != theta_initial:
        A = (theta_initial - theta_final)/(2*(tf**3))
        B = -3*tf*A
    else:
        A = 0
        B = 0
    l = [ tf, A, B]
    return l

def execute(goal):
    global pub,jnt,rate,begin
    theta_in = goal.current_pose
    theta_f = goal.goal
    vel_lmts = [0.6,0.6,0.6,0.6]
    i = [0,0,0,0]
    s1 = eq_params(vel_lmts[0],theta_f[0],theta_in[0])
    s2 = eq_params(vel_lmts[1],theta_f[1],theta_in[1])
    s3 = eq_params(vel_lmts[2],theta_f[2],theta_in[2])
    s4 = eq_params(vel_lmts[3],theta_f[3],theta_in[3])
    time_initial = rospy.get_time()
    while True:
        jnt.header.stamp = rospy.Time.now()
        current_time = rospy.get_time()
        delta_t = current_time - time_initial
        ct2 = delta_t**2
        ct3 = ct2*delta_t
        if goal.gripper == 1:
            j5 =  2300
        elif goal.gripper == 0:
            j5 = 1800
        else:
            j5 = 2000
        if (s1[0] > delta_t):
            j1 = s1[1]*ct3 + s1[2]*ct2 + theta_in[0]
        elif s1[0] == 0:
            j1 = theta_in[0]
            i[0] = 1
        else:
            i[0] = 1

        if (s2[0] > delta_t):
            j2 = s2[1]*ct3 + s2[2]*ct2 + theta_in[1]
        elif s2[0] == 0:
            i[1] = 1
            j2 = theta_in[1]
        else:
            i[1] = 1

        if (s3[0] > delta_t):
            j3 = s3[1]*ct3 + s3[2]*ct2 + theta_in[2]
        elif s3[0] == 0:
            i[2] = 1
            j3 = theta_in[2]
        else:
            i[2] = 1

        if (s4[0] > delta_t):
            j4 = s4[1]*ct3 + s4[2]*ct2 + theta_in[3]
        elif s4[0] == 0:
            i[3] = 1
            j4 = theta_in[3]
        else:
            i[3] = 1
        if server.is_preempt_requested():
            server.set_preempted("client requested to stop")
            return
        if i[0] == 1 and i[1]==1 and i[2]==1 and i[3]==1:
            break
        feedback = joint_controlFeedback()
        feedback.status = [j1,j2,j3,j4]
        server.publish_feedback(feedback)
        jnt.position = [j1,j2,j3,j4,j5]
        rate.sleep()
        pub.publish(jnt)
    r = joint_controlResult()
    r.result = 1
    begin =0
    server.set_succeeded(r, "goal reached successfully")
rospy.init_node('arm_control_server')
rate = rospy.Rate(90)
server = actionlib.SimpleActionServer('hardware_control', joint_controlAction, execute, False)
server.start()
while True:
    if not begin:
        jnt.header.stamp = rospy.Time.now()
        pub.publish(jnt)
        rate.sleep()