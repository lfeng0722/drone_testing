#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry


def callback_ned_odom(data):
    # Create a new Odometry message for ENU data
    enu_odom = Odometry()

    # Header
    enu_odom.header = data.header
    enu_odom.header.frame_id = "odom_enu"  # change to your desired frame name

    # Position
    enu_odom.pose.pose.position.x = data.pose.pose.position.y
    enu_odom.pose.pose.position.y = data.pose.pose.position.x
    enu_odom.pose.pose.position.z = -data.pose.pose.position.z

    # Orientation (assuming quaternion representation)
    # For this example, only position is transformed.
    # For a complete transformation, consider using transformations from tf package

    enu_odom.pose.pose.orientation = data.pose.pose.orientation

    # Velocities if present
    enu_odom.twist.twist.linear.x = data.twist.twist.linear.y
    enu_odom.twist.twist.linear.y = data.twist.twist.linear.x
    enu_odom.twist.twist.linear.z = -data.twist.twist.linear.z

    enu_odom.twist.twist.angular = data.twist.twist.angular

    # Publish the ENU Odometry message
    pub.publish(enu_odom)


if __name__ == '__main__':
    rospy.init_node('ned_to_enu_converter', anonymous=True)
    rospy.Subscriber("/airsim_node/Copter/odom_local_ned", Odometry, callback_ned_odom)

    pub = rospy.Publisher('/odom_enu', Odometry, queue_size=10)

    rospy.spin()
