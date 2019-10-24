#! /usr/bin/env python
# -*- coding: utf-8 -*-

from threading import Lock

import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

class FollowLines():
    
    """
    Class dedicated for make a robot follow two parallel lines.

    Prerequisite:
    The robot has to start in a position between the two lines. In any other case, an additional algorithm should be implemented for getting the robot to a position between the two lines.
    """

    _LINEAR_VEL = 0.4
    _ANGULAR_VEL = 0.3

    def __init__(self):
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        """Subscription for the detection topic of both sensors"""
        self._sub_left_sensor = rospy.Subscriber('/color_sensor_plugin/left_color_sensor', Bool, self._left_sensor_cb)
        self._sub_right_sensor = rospy.Subscriber('/color_sensor_plugin/right_color_sensor', Bool, self._right_sensor_cb)
        """ Two booleans for the detection of both sensors"""
        self._left_line_detected = False
        self._right_line_detected= False
        self._twist = Twist()
        self._lock = Lock()
        self._hz = rospy.get_param('~hz', 10)
        self._rate = rospy.Rate(self._hz)
        """ Waiting time until checking if rotation is needed after starting going forward"""
        self._forward_rate = rospy.Rate(100)
        """ Counter for amount of rotations without going forward. Avoids the robot getting stuck."""
        self._rotation_count = 0


    def _left_sensor_cb(self,data):
        rospy.loginfo("left sensor cb")
        self._left_line_detected = data.data

    def _right_sensor_cb(self,data):
        rospy.loginfo("right sensor cb")
        self._right_line_detected = data.data

    def _publish(self,twist):
        self._pub_cmd.publish(twist)

    def _go_forward(self):
        rospy.loginfo("forward")
        self._twist.linear.x = self._LINEAR_VEL
        self._twist.angular.z = 0
        self._publish(self._twist)
        """self._forward_rate.sleep()"""
        self._rotation_count = 0

    def _rotate_left(self):
        rospy.loginfo("rotate left")
        self._twist.angular.z = self._ANGULAR_VEL
        self._twist.linear.x = 0
        self._publish(self._twist)      
        self._rotation_count += 1

    def _rotate_right(self):
        rospy.loginfo("rotate right")
        self._twist.angular.z = -1.0 * self._ANGULAR_VEL
        self._twist.linear.x = 0
        self._publish(self._twist)  
        self._rotation_count += 1     

    def run(self):

        """
        Executes the global algorithm for following the lines.
        """
        rospy.loginfo("run")
        while not rospy.is_shutdown():
            if(self._rotation_count > 8):
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