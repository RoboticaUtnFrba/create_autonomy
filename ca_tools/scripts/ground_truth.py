#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math
from threading import Lock

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry


class GroundTruth():


    """This class provides methods for easy accessing the ground truth system native to Gazebo simulator.
    """

    def __init__(self):
        self._sub_gts=rospy.Subscriber('gts', Odometry, self._callback)        
        self._odometry = Odometry()
        self._lock = Lock()

    def _callback(self,data):
        self._lock.acquire()
        self._odometry = data
        self._lock.release()

    def set_frame_id(self,frame):
        self._frame = frame

    def get_pose(self):
        return self._odometry.pose.pose

    def get_twist(self):
        return self._odometry.twist.twist

    def get_header(self):
        return self._odometry.header
