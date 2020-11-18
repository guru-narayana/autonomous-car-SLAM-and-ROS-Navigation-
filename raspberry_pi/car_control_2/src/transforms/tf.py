#!/usr/bin/env python
import roslib
roslib.load_manifest('car_control_2')
import rospy
import geometry_msgs.msg
import tf2_ros as tf
t = geometry_msgs.msg.TransformStamped()
rospy.init_node('fixed_tf_broadcaster')
br = tf.TransformBroadcaster()
while not rospy.is_shutdown():
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "laser_frame"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1
    br.sendTransform(t)
