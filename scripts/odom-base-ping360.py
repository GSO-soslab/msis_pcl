#!/usr/bin/env python3
 
import rospy
import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg._TransformStamped 
from nav_msgs.msg import Odometry #Message type. Depends on what kind of data you're working with.
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def callbakc_pcl(msg):
    print("hello")
    br = tf2_ros.TransformBroadcaster()

    s =geometry_msgs.msg.TransformStamped()
    s.header.frame_id = "alpha_rise/odom"
    s.child_frame_id = "alpha_rise/base_link"
    s.header.stamp = msg.header.stamp

    s.transform.translation.x = msg.pose.pose.position.x
    s.transform.translation.y = msg.pose.pose.position.y
    s.transform.translation.z = msg.pose.pose.position.z

    s.transform.rotation.x = msg.pose.pose.orientation.x
    s.transform.rotation.y = msg.pose.pose.orientation.y
    s.transform.rotation.z = msg.pose.pose.orientation.z
    s.transform.rotation.w = msg.pose.pose.orientation.w


    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = msg.header.stamp
    t.header.frame_id = "alpha_rise/base_link"
    t.child_frame_id  = "alpha_rise/ping360_link"

    t.transform.translation.x = 0.792
    t.transform.translation.y = 0
    t.transform.translation.z = -0.085 
    q = tf_conversions.transformations.quaternion_from_euler(3.14, 3.14, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)
    br.sendTransform(s)

    # listener = tf.TransformListener() #listens to the tf thrown
    # (trans,rot) = listener.lookupTransform('alpha_rise/odom', 'alpha_rise/base_link', msg.header.stamp)

if __name__ == "__main__":
    rospy.init_node('tf_listener')
    rospy.Subscriber("/alpha_rise/odometry/filtered/local", Odometry, callbakc_pcl)

    # rospy.Subscriber("/msis/pointcloud/cpp", PointCloud2, callbakc_pcl)
    rospy.spin()