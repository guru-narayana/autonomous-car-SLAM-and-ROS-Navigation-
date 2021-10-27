#!/usr/bin/env python
# importing nesscessry modules
import rospy
import math
from math import sin, cos, pi
import tf_conversions
import signal
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose,Twist,Quaternion,Vector3,TransformStamped
# setting the parametrs for position of G_bot(turtle_bot)
x = 0.0
y = 0.0
theta = 0.0
t = TransformStamped()
def odom_callback(vel):
    odom_pub = rospy.Publisher("odom",Odometry,queue_size=10)
    odom_broadcaster = tf2_ros.TransformBroadcaster()
    global x,y,theta,current_time,past_time
    # setting the current time and time passed
    current_time = rospy.Time.now()
    dt = (current_time-past_time).to_sec()
    # calculating distance using velocities
    v = vel.linear.x
    vtheta = -vel.angular.z
    x += (v * cos(theta))* dt
    y += (v * sin(theta)) * dt
    theta += vtheta * dt
    past_time = current_time
    # transforming the eulars into quaternions
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
    # publishing the transform between /odom and base_link
    t.header.stamp = current_time
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    #defining the odometry message
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    # set the position
    odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*q))
    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, vtheta))
    # publish the message
    odom_broadcaster.sendTransform(t)
    odom_pub.publish(odom)
# initialising the node
rospy.init_node("odometry_publisher",anonymous=True)
current_time = rospy.Time.now()
past_time = rospy.Time.now()
rospy.Subscriber("/g_cmd_vel", Twist, odom_callback)
rospy.spin()
