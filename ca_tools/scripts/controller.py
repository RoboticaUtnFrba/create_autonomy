#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math
from robot_localization_tf import RobotLocalizationTf
from threading import Lock
from ground_truth import GroundTruth

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion as efq




class PoseController():

    """Class that implements a controller for the pose of a robot.
    """

    _INTEGRAL_TERMS = 10000

    def __init__(self, pub_topic='cmd_vel_control', goal_pose_topic='goal_pose', switch_topic='controller_on', kp_angle=0.1, ki_angle=0.0, kd_angle=0.0, kp_vel=0.1, ki_vel=0.0, kd_vel=0.0):
        self._current_pose = Pose()
        self._goal_pose = Pose()
        self._pub_cmd = rospy.Publisher(pub_topic, Twist, queue_size=10)
        self._goal_pose_sub = rospy.Subscriber(goal_pose_topic, Pose, self._goal_pose_callback)
        self._switch = rospy.Subscriber(switch_topic, Bool, self._on_callback) # This topic tells the controller when to send twist messages
        self._rate = rospy.Rate(2)
        self._tf = RobotLocalizationTf()
        self._kp_angle = kp_angle
        self._ki_angle = ki_angle
        self._kd_angle = kd_angle
        self._kp_vel = kp_vel
        self._ki_vel = ki_vel
        self._kd_vel = kd_vel
        self._turned_on = False
        self._angle_diff = 0
        self._integral_angle_error = 0
        self._integral_position_error = 0
        self._dt = 0.00001                   # Differential for computing integral and derivative error
        self._lock = Lock()
        self._integral_count = 0             # Variable for counting integral terms summed until setting it back to zero
        self._last_angle_diff = 0            # Variable for computing the error difference for the derivative controller of the angle
        self._last_position_diff = 0         # Variable for computing the error difference for the derivative controller of the position
        self._ground_truth = GroundTruth()
        self._BASE_VEL = 0.3                 # Constant additive coefficient of velocity for not having zero linear velocity when the robot is very close to his goal

    def _goal_pose_callback(self, data):
        self._lock.acquire()
        self._goal_pose = data
        _, _, self._angle_diff,_ = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
        self._lock.release()

    def _on_callback(self, data):
        self._lock.acquire()
        self._turned_on = data.data
        self._lock.release()

    def set_constants(self,kp_angle=0.1, ki_angle=0.1, kd_angle=0.0, kp_vel=0.1, ki_vel=0.1,kd_vel=0.0):

        """Sets the constants for the angle and velocity controllers
        """

        self._kp_angle = kp_angle
        self._ki_angle = ki_angle
        self._kd_angle = kd_angle
        self._kp_vel = kp_vel
        self._ki_vel = ki_vel
        self._kd_vel = kd_vel


    def _publish_vel(self, vel):
        self._pub_cmd.publish(vel)

    def _compute_velocity(self):

        """Computes the twist message based on the pose differences between the current pose and goal pose.
        
        Returns:
            ROS Twist message
        """

        self._current_pose = self._ground_truth.get_pose()
        _, _, self._angle_diff, self._length_diff = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
        self._integral_angle_error += self._angle_diff * self._dt
        self._integral_position_error += self._length_diff * self._dt
        self._derivative_angle = (self._angle_diff - self._last_angle_diff)
        self._derivative_position = (self._length_diff - self._last_position_diff) / self._dt
        self._last_angle_error = self._angle_diff
        self._last_position_error = self._length_diff
        angular = self._angle_diff * self._kp_angle + self._integral_angle_error * self._ki_angle + self._derivative_angle * self._kd_angle
        linear =  self._BASE_VEL + self._length_diff * self._kp_vel + self._integral_position_error * self._ki_vel + self._derivative_position * self._kd_vel
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._integral_count += 1
        if(self._integral_count > self._INTEGRAL_TERMS):
            self._integral_count = 0
        return twist

    def _PID(self):
        twist = self._compute_velocity()
        self._publish_vel(twist)

    def run(self):
        while not rospy.is_shutdown():
            if(self._turned_on):
                self._PID()
            self._rate.sleep()

def main():
    rospy.init_node('pose_controller')
    app = PoseController()
    app.set_constants(1, 0.5, -0.1, 0.1, 1)
    app.run()

main()
