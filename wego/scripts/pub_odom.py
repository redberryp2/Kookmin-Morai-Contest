#! /usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry

class PubTF:
    def __init__(self):
        rospy.init_node('odom_tf_broadcast', anonymous=True)
        self.br = tf2_ros.TransformBroadcaster()
        self.TransformStamped = TransformStamped()
        self.TransformStamped.header.frame_id = "odom"

        rospy.Subscriber('/odom', Odometry, self.callback)

    def callback(self, msg):
        self.TransformStamped.child_frame_id =  msg.child_frame_id
        self.TransformStamped.header.stamp =  rospy.Time.now()
        self.TransformStamped.transform.translation.x = msg.pose.pose.position.x
        self.TransformStamped.transform.translation.y = msg.pose.pose.position.y
        self.TransformStamped.transform.translation.z = msg.pose.pose.position.z
        self.TransformStamped.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(self.TransformStamped)


def main():
    try:
        Pub_tf =PubTF()
        rospy.spin()
    except rospy.ROSInternalException:
        pass

    
    
if __name__ == "__main__":
    main()