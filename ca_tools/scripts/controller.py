#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math
from RobotLocalizationTf import RobotLocalizationTf
from threading import Lock
from GTSys import GroundTruth

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion as efq




class PoseController():

    """Class that implements a controller for the pose of a robot.
    """

    _INTEGRAL_TERMS = 10000

    def __init__(self, pub_topic='cmd_vel_control', kp=0.1, ki=0, kd=0):
        self._current_pose = Pose()
        self._goal_pose = Pose()
        self._pub_cmd = rospy.Publisher(pub_topic, Twist, queue_size=10)
        self._rate = rospy.Rate(10)
        self._tf = RobotLocalizationTf()
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._turned_on = False
        self._angle_diff = 0
        self._integral_error = 0
        self._dt = 0.00001        # differential for computing integral error
        self._lock = Lock()
        self._integral_count = 0 # Variable for counting integral terms summed until setting it back to zero
        self._ground_truth = GroundTruth()

    '''def _current_pose_callback(self, data):
        self._lock.acquire()
        self._current_pose = data
        _, _, self._angle_diff,_ = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
        self._lock.release()'''

    def _goal_pose_callback(self, data):
        self._lock.acquire()
        self._goal_pose = data
        _, _, self._angle_diff,_ = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
        self._lock.release()

    def _on_callback(self, data):
        self._lock.acquire()
        self._turned_on = data.data
        self._lock.release()

    def set_pose_subscribers(self, current_pose_topic='current_pose', goal_pose_topic='goal_pose'):
        '''self._current_pose_sub = rospy.Subscriber(current_pose_topic, Pose, self._current_pose_callback)'''
        self._goal_pose_sub = rospy.Subscriber(goal_pose_topic, Pose, self._goal_pose_callback)
        self._switch = rospy.Subscriber('controller_on', Bool, self._on_callback)

    def set_constants(self,kp=0.1,ki=0,kd=0):
        self._kp = kp
        self._ki = ki
        self._kd = kd

    def _publish_vel(self, vel):
        self._pub_cmd.publish(vel)

    def _compute_velocity(self):
        self._current_pose = self._ground_truth.get_pose()
        _, _, self._angle_diff, self._length_diff = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
        rospy.loginfo("Contoller angle diff: %f", self._angle_diff)
        self._integral_error += self._angle_diff * self._dt
        angular = self._angle_diff * self._kp + self._integral_error * self._ki
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = angular
        self._integral_count += 1
        if(self._integral_count > self._INTEGRAL_TERMS):
            self._integral_count = 0
        return twist

    def _PID(self):
        twist = self._compute_velocity()
        self._publish_vel(twist)

    def run(self):
        while True:
            if(self._turned_on == True):
                self._PID()
            self._rate.sleep()

def main():
    rospy.init_node('pose_controller')
    app = PoseController('cmd_vel_control', 1, 0.5)
    app.set_pose_subscribers()
    app.run()

main()
