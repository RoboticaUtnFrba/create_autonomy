#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from enum import Enum
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool, Int64

class RobotState(Enum):

    """
    Enumerative type for states of the robot.
    """

    STOP = 1
    MOVE_FORWARD = 2
    ROTATE_LEFT = 3
    ROTATE_RIGHT = 4

class FollowLines():
    
    """
    Class dedicated for make a robot follow two parallel lines.

    Prerequisite:
    The robot has to start in a position between the two lines. In any other case, an additional algorithm should be implemented for getting the robot to a position between the two lines.
    """

    _LINEAR_VEL = 0.25
    _ANGULAR_VEL = 0.3
    """ Amount of rotations until the robot is considered stuck """
    _MAX_ROTATIONS = 6

    def __init__(self):
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._pub_count = rospy.Publisher('rotation_count', Int64, queue_size=10)
        """ Subscription for the detection topic of both sensors """
        self._sub_left_sensor = rospy.Subscriber('/color_sensor_plugin/left_color_sensor', Bool, self._left_sensor_cb)
        self._sub_right_sensor = rospy.Subscriber('/color_sensor_plugin/right_color_sensor', Bool, self._right_sensor_cb)
        """ Two booleans for the detection of both sensors """
        self._left_line_detected = False
        self._right_line_detected= False
        self._twist = Twist()
        self._rate = rospy.Rate(20)
        """ Counter for amount of rotations without going forward. Avoids the robot getting stuck """
        self._rotation_count = 0
        """ Current state of the robot """
        self._state = RobotState.STOP

    def _left_sensor_cb(self,data):
        self._left_line_detected = data.data

    def _right_sensor_cb(self,data):
        self._right_line_detected = data.data

    def _publish(self,twist):
        self._pub_cmd.publish(twist)

    def _go_forward(self):
        self._twist.linear.x = self._LINEAR_VEL
        self._twist.angular.z = 0
        self._publish(self._twist)
        self._rotation_count = 0
        self._state= RobotState.MOVE_FORWARD

    def _rotate_left(self):
        self._twist.angular.z = self._ANGULAR_VEL
        self._twist.linear.x = 0
        self._publish(self._twist)    
        if(self._state != RobotState.ROTATE_LEFT):
            self._rotation_count += 1
            self._state = RobotState.ROTATE_LEFT

    def _rotate_right(self):
        self._twist.angular.z = -1.0 * self._ANGULAR_VEL
        self._twist.linear.x = 0
        self._publish(self._twist)  
        self._rotation_count += 1    
        if(self._state != RobotState.ROTATE_RIGHT):
            self._rotation_count += 1
            self._state = RobotState.ROTATE_RIGHT 

    def run(self):

        """
        Executes the global algorithm for following the lines.
        """

        rospy.loginfo("run")
        while not rospy.is_shutdown():
            if(self._rotation_count > self._MAX_ROTATIONS):
                self._go_forward()
            elif(self._right_line_detected and self._left_line_detected):
                self._go_forward()
            elif(self._right_line_detected):
                self._rotate_left()
            elif(self._left_line_detected):
                self._rotate_right()
            self._rate.sleep()
        rospy.spin()

def main():
    rospy.init_node('follow_lines')
    app = FollowLines()
    app.run()

main()
