#!/usr/bin/env python3

import rospy
import math
import sys
import tf2_ros

from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

# subscriber topics

gazebo_odom_topic = '/odom'
odom_frame = '/odom'
base_frame = '/base_footprint'

# information publishers

tf_pub        = tf2_ros.TransformBroadcaster()

# footprint parameters

def odom_callback(data):

    odom                      = Odometry()
    odom.header.frame_id      = odom_frame
    odom.child_frame_id       = base_frame
    odom.header.stamp         = rospy.Time.now()
    odom.pose                 = data.pose
    odom.twist = data.twist

    tf = TransformStamped(header         = Header(
                          frame_id       = odom.header.frame_id,
                          stamp          = odom.header.stamp),
                          child_frame_id = odom.child_frame_id,
                          transform      = Transform(
                          translation    = odom.pose.pose.position,
                          rotation       = odom.pose.pose.orientation))

    # visualize footprint everytime odom changes

    tf_pub.sendTransform(tf)

if __name__ == '__main__':

    try:

        rospy.init_node('control_plugin', anonymous = True)

        rospy.Subscriber(gazebo_odom_topic, Odometry, odom_callback)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
