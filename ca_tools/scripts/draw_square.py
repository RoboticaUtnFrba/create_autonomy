#! /usr/bin/env python
# -*- coding: utf-8 -*-


import curses
import math

import rospy
from geometry_msgs.msg import Twist

class DrawSquare():

    _init_pose = None
    _linear = None
    _angular = None

    def __init__(self,first_pt,second_pt,third_pt)
        self._interface = interface
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)

    def _publish()
        twist = Twist()
        twist.linear.x = 0.7
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self._pub_cmd.publish(twist)

    def run(self)
        self._linear = 0
        self._angular = 0

        self._publish()



def main():
    rospy.init_node('draw_square')
    app = DrawSquare()
    app.run()