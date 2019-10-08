#! /usr/bin/env python
# -*- coding: utf-8 -*-



# A node just made for testing. It's only goal is to publish constantly the current pose of the robot, as measured by the ground truth system
#############################################################################################################################################

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from GTSys import GroundTruth

def main():
    rospy.init_node('test_pose')
    pub_current = rospy.Publisher('current_pose', Pose, queue_size=10)
    gt = GroundTruth()

    while True:
        pose = gt.get_pose()
        pub_current.publish(pose)


main()