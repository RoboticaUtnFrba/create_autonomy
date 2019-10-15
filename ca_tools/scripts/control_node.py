#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) Ekumen
# Released under the BSD License.
#
# Authors:
#   * Ramiro Serra

import math
import signal
import sys
from math import pi
from enum import Enum

import rospy
from geometry_msgs.msg import Twist, Pose2D, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion




LINEAR_VEL = 0.2
DISTANCE_TOLERANCE = 0.1
ANGLE_TOLERANCE = 0.2
ANGULAR_VEL = 1.0
VEL_PUB_TOPIC = "/create1/cmd_vel"
GTS_SUBTOPIC = "/create1/gts"
BIG_ANGLE_THRESHOLD = 1.0  # minimum angle in which the robot will stop and turn


class STATES(Enum): FORWARD=1;STOP_MOVING_FORWARD=2;TURNING=3;STOP_TURNING=4


def signal_handler(signal, frame, queue_size=1):
  # This will execute when Ctrl-c is pressed
    pub = rospy.Publisher(VEL_PUB_TOPIC, Twist, queue_size=1)
    aux = Twist();pub.publish(aux);sys.exit(0)


class CtrlNode(object):
    """Base class for controlling the robot, defines the state machine
    and some common methods. As well as a trivial implementation of the problem
    
    
    """

    def __init__(self):
        """Initializations
        """

        rospy.init_node('control_node')
        
        self._my_vel_pub = rospy.Publisher(VEL_PUB_TOPIC, Twist, queue_size=10)
        # Create a subscriber with appropriate topic, custom message and name of callback function.
        self._my_sub = rospy.Subscriber(
            GTS_SUBTOPIC, Odometry, self._gts_callback)
        self._my_goals = [Pose2D()] * 4
        self._my_goals[1] = Pose2D(0, 2, 0)
        self._my_goals[2] = Pose2D(2, 2, 0)
        self._my_goals[3] = Pose2D(2, 0, 0)

        self._goal_num = 0
        self._state = STATES.STOP_MOVING_FORWARD
        self._my_pose = Pose2D()
        self._my_pose.x = None
        self._my_pose.y = None
        self._my_angle_goal = 0.0

    def _gts_callback(self, data):
        """Callback to groundtrouth
        
        Arguments:
            data {[Odometry msg]} -- [Odometry by gts]
        """
        self._my_pose.x = data.pose.pose.position.x
        self._my_pose.y = data.pose.pose.position.y
        q = data.pose.pose.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        (_,_,self._my_pose.theta) = euler_from_quaternion(q_arr)

    def move(self):
        """State machine
        """
        if not (self._my_pose.x == None or self._my_pose.y == None):
            if self._state == STATES.TURNING:
                self._turning()
                return
            if self._state == STATES.STOP_TURNING:
                self._stop_turning()
                return
            if self._state == STATES.FORWARD:
                self._forward()
                return
            if self._state == STATES.STOP_MOVING_FORWARD:
                self._stop_moving_forward()
                return

    def _turning(self):

        if (self._reached_angle()):
            self._stop()
            self._state = STATES.STOP_TURNING

        else:
            aux = Twist()

            if (self._diff_angle()) < 0:
                aux.angular.z = - ANGULAR_VEL
            else:
                aux.angular.z = ANGULAR_VEL

            self._my_vel_pub.publish(aux)

    def _stop_turning(self):
        self._stop()
        self._move_forward()
        self._state = STATES.FORWARD

    def _stop(self):
        aux = Twist()
        self._my_vel_pub.publish(aux)

    def _move_forward(self):
        aux = Twist()
        aux.linear.x = LINEAR_VEL
        self._my_vel_pub.publish(aux)

    def _forward(self):
        
        if (self._reached_position()):
            self._stop()
            self._state = STATES.STOP_MOVING_FORWARD
            self._get_next_goal()

        else:
            self._set_goal_angle()
            if (not self._reached_angle()):
                self._state = STATES.STOP_MOVING_FORWARD
                self._stop()
            else:
                self._move_forward()

    def _get_next_goal(self):
        self._goal_num += 1
        self._goal_num %= len(self._my_goals)

    def _diff_angle(self):
        """Calculates the difference between the angle goal and current angle

        Returns:
            [float] -- [Difference of angles in radians, wrapped between -pi/2 and pi/2]
        """
        auxangle = (self._my_angle_goal - self._my_pose.theta)
        # wrapping angle
        return math.atan2(math.sin(auxangle), math.cos(auxangle))

    def _angle_is_big(self):
        """Return if the angle difference between robot and goal is greater than a threshold
        
        Returns:
            [boolean] -- [description]
        """
        return (abs(self._diff_angle()) > BIG_ANGLE_THRESHOLD)

    def _stop_moving_forward(self):
        """Sets new goal
        """
        self._stop()
        self._set_goal_angle()
        self._state = STATES.TURNING

    def _set_goal_angle(self):
        """Sets the goal angle with the current position and the goal position
        """
        self._my_angle_goal = math.atan2(
            (self._my_goals[self._goal_num].y - self._my_pose.y), self._my_goals[self._goal_num].x - self._my_pose.x)

    def _reached_angle(self):
        """Returns true if reached angle
        
        Returns:
            [bool] -- [description]
        """
        return (abs(self._diff_angle()) < ANGLE_TOLERANCE)

    def _reached_position(self):
        """Returns true if reached position
        
        Returns:
            [bool] -- [description]
        """
        return (self._diff_distance() < DISTANCE_TOLERANCE)


    def _diff_distance(self):
        """Returns the distance between the robot and the goal
        """
        return(math.hypot(self._my_goals[self._goal_num].x - self._my_pose.x, self._my_goals[self._goal_num].y - self._my_pose.y))


if __name__ == '__main__':
    node = CtrlNode()
    # associates signal with handler
    signal.signal(signal.SIGINT, signal_handler)
    while not rospy.is_shutdown():
        rospy.sleep(0.3)
        node.move()
