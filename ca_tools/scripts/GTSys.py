#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry


class GTSys():

    _pose = Pose()
    _twist = Twist()

    def __init__(self):
        self._sub_gts=rospy.Subscriber('gts', Odometry, self._callback)

    def _callback(self,data):
        self._pose = data.pose.pose
        self._twist = data.twist.twist
        self._header = data.header

    def set_frame_id(self,frame):
        self._header.frame_id = frame

    def get_pose(self):
        return self._pose

    def get_twist(self):
        return self._twist

    def get_header(self):
        return self._header