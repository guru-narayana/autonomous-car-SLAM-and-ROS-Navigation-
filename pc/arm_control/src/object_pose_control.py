#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose,TransformStamped
import tf_conversions
import tf2_ros
t = TransformStamped()
object_pos_broadcaster = tf2_ros.TransformBroadcaster()
def Pose_callback(pos):
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera"
    t.child_frame_id = "object"
    t.transform.translation.x = pos.position.x/100
    t.transform.translation.y = pos.position.y/100+0.02
    t.transform.translation.z = pos.position.z/100
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    object_pos_broadcaster.sendTransform(t)


rospy.init_node("object_control",anonymous=True)
current_time = rospy.Time.now()
past_time = rospy.Time.now()
rospy.Subscriber("/object_pose", Pose, Pose_callback)
rospy.spin()