#! /usr/bin/env python
import rospy
import actionlib
import time
from bot_msgs.msg import joint_controlAction, joint_controlGoal, joint_controlResult, joint_controlFeedback
from sensor_msgs.msg import JointState
from bot_msgs.srv import Inverse_kinematics,Inverse_kinematicsResponse

rospy.init_node('arm_initiation_node')
client = actionlib.SimpleActionClient('hardware_control', joint_controlAction)
client.wait_for_server()
jnt_position = [] #varible that stores the current joint states
pr = 0 # variable to make sure the actual joint position is recived atleast once
# feedback function for testing the collision which is calculated 
# by using difference between actual joint position and position sent by the action server.
def feedback_cb(feedback):
    global jnt_position
    for i in range(4):
        if abs(abs(feedback.status[i]) - abs(jnt_position[i])) > 10:
            string1 = "Error at joint no: "+str(i+1)+" joint cancelling the goal"
            string2 = str(feedback.status[i]) + "   "+str(jnt_position[i])
            rospy.loginfo(string1 +string2)
            client.cancel_goal()
#action-control function that makes the goal request to action server     
def action_control(theta_I,theta_F,grip = 0):
    global pr
    if pr==1:
        goal = joint_controlGoal()
        goal.current_pose = theta_I
        goal.goal = theta_F
        goal.gripper = grip
        client.send_goal(goal, feedback_cb=feedback_cb)
        client.wait_for_result()
# callback from current joint position subscriber 
def current_pos_callback(jnt):
    global jnt_position,pr
    pr = 1
    jnt_position =  jnt.position
# to make the arm go into active state
def active(i=0):
    global jnt_position
    theta_I = jnt_position
    theta_F = [-1.57,-1,1.57,1.57]
    action_control(theta_I,theta_F)
    theta_I = [-1.57,-1,1.57,1.57]
    theta_F = [0,-1.22,1.22,0]
    action_control(theta_I,theta_F,i)

# to bring the arm into its home poistion  
def home():
    global jnt_position
    theta_I = jnt_position
    theta_F = [-1.57,-1,1.57,1.57]
    action_control(theta_I,theta_F)
    theta_I = [-1.57,-1,1.57,1.57]
    theta_F = [-1.57,-0.1,1.57,1.57]
    action_control(theta_I,theta_F)
    jnt_position =theta_F
def Ik_to_arm_cb(jnt1):
    global jnt_position
    active(1)
    Theta_I = [0,-1.22,1.22,0]
    Theta_F = [jnt1.s1,jnt1.s2,jnt1.s3,jnt1.s4]
    action_control(Theta_I,Theta_F,1)
    Theta_I = [jnt1.s1,jnt1.s2,jnt1.s3,jnt1.s4]
    Theta_F = [0,-1.22,1.22,0]
    time.sleep(1)
    action_control(Theta_I,Theta_F)
    return Inverse_kinematicsResponse(1)

rospy.Subscriber("Servo_joint_states", JointState, current_pos_callback)
while pr == 0:
    pass
home()
s = rospy.Service('pose_to_arm_ontrol', Inverse_kinematics, Ik_to_arm_cb)
rospy.spin()