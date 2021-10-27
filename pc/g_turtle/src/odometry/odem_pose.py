#!/usr/bin/env python
# importing nesscessry modules
import rospy
import math
import tf_conversions
import signal
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose,Twist,Quaternion,Vector3,TransformStamped
# setting the parametrs for position of G_bot(turtle_bot)
x = 0.0
y = 0.0
x_p = 0.0
y_p = 0.0
theta_p = 0.0
v = 0.0
v_theta = 0.0
t = TransformStamped()
def odom_callback(pose):
    odom_pub = rospy.Publisher("odom",Odometry,queue_size=10)
    odom_broadcaster = tf2_ros.TransformBroadcaster()
    global x,y,theta,current_time,past_time,x_p,y_p,theta_p,v,v_theta
    current_time = rospy.Time.now()
    dt = (current_time-past_time).to_sec()
    x = pose.position.x
    y = pose.position.y
    orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    (roll, pitch, yaw) = tf_conversions.transformations.euler_from_quaternion (orientation_list)
    if dt > 1:
    	v = ((x-x_p)/dt)*math.cos((yaw*180)/3.14) + ((y-y_p)/dt)*math.sin((yaw*180)/3.14)
    	v_theta = (yaw-theta_p)/dt
    	x_p = x
    	y_p = y
    theta_p = yaw
    t.header.stamp = current_time
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = orientation_list[0]
    t.transform.rotation.y = orientation_list[1]
    t.transform.rotation.z = orientation_list[2]
    t.transform.rotation.w = orientation_list[3]
    odom_broadcaster.sendTransform(t)
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    # set the position
    odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*orientation_list))
    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, v_theta))
    # publish the message
    odom_pub.publish(odom)
    rospy.loginfo(v)
	
# initialising the node
rospy.init_node("odometry_publisher",anonymous=True)
current_time = rospy.Time.now()
past_time = rospy.Time.now()
rospy.Subscriber("g_Pose", Pose, odom_callback)
rospy.spin()
