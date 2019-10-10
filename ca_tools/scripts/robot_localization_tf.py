#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
from geometry_msgs.msg import Twist, Pose, Quaternion
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe


class RobotLocalizationTf:


    """Class that provides methods for different computations regarding localization of robots.
    """


    @staticmethod
    def wrap_angle(angle, min_angle=-1.0*math.pi, max_angle=math.pi): #ToDo: make beautiful this horrible function
        while (angle<min_angle):
            angle+=2.0*math.pi
        while (angle>max_angle):
            angle-=2.0*math.pi
        return angle

    @staticmethod
    def get_pose_diff(current_pose, goal_pose):

        """Computes position and angle difference between two poses
        
        Returns:
            Tuple with:
                difference in x axis
                difference in y axis
                angle difference (in radians)
                distance between positions
        """

        q = current_pose.orientation
        _, _, yaw = efq((q.x, q.y, q.z, q.w))
        current_angle = RobotLocalizationTf.wrap_angle(yaw)

        diff_x = (goal_pose.position.x - current_pose.position.x)
        diff_y = (goal_pose.position.y - current_pose.position.y)

        goal_angle = math.atan2(diff_y, diff_x)
        angle_diff = RobotLocalizationTf.wrap_angle(goal_angle - current_angle)
        length_diff = math.hypot(diff_x, diff_y)

        return (diff_x, diff_y, angle_diff, length_diff)
