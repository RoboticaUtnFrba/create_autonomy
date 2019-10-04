#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math

import rospy
from GTSys import GTSys
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

class pathPrinter():

    def __init__(self):
        self._path = Path()
        self._path_pub = rospy.Publisher('path', Path, queue_size=10)
        self._gt = GTSys()
        self._rate = rospy.Rate(1) #1 Path msg per second
        self._rate.sleep()

    def _update_path(self):
        self._current_header = self._gt.get_header()
        self._current_pose = self._gt.get_pose()
        pose = PoseStamped()
        pose.header = self._gt.get_header()
        pose.pose = self._gt.get_pose()
        self._path.poses.append(pose)


    def run(self):
        while True:
            self._update_path()
            self._path_pub.publish(self._path)
            self._rate.sleep()

def main():
    rospy.init_node('path_printer')
    app = pathPrinter()
    app.run()

main()