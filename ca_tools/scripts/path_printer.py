#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math
import rospy
from ground_truth import GroundTruth
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path


class PathPrinter():


    """
    Class that sends a Path msg to a desired topic, and updates the message as time passes.
    """

    def __init__(self, pub_topic="path", frame_id="/map", pub_rate=1):
        self._path = Path()
        self._path_pub = rospy.Publisher(pub_topic, Path, queue_size=10)
        self._gt = GroundTruth()
        self._rate = rospy.Rate(pub_rate)
        self._path.header.frame_id = frame_id
        self._gt.set_frame_id(frame_id)

    def _update_path(self):
        self._current_header = self._gt.get_header()
        self._current_pose = self._gt.get_pose()
        pose = PoseStamped()
        pose.header = self._gt.get_header() # Sets the correct header for printing in RViz
        pose.pose = self._gt.get_pose()
        self._path.poses.append(pose)

    def run(self):
        while True:
            self._update_path()
            self._path_pub.publish(self._path)
            self._rate.sleep()


def main():
    rospy.init_node('path_printer')
    app = PathPrinter('path', '/map', 1)
    app.run()

main()
