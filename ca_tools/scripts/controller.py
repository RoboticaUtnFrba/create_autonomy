#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math
from RobotLocalizationTf import RobotLocalizationTf

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion as efq




class PoseController():

    def __init__(self, velocity_publisher_topic = 'cmd_vel'):
        self._current_pose = Pose()
        self._goal_pose = Pose()
        self._pub_cmd = rospy.Publisher(velocity_publisher_topic, Twist, queue_size=10)
        self._hz = rospy.get_param('~hz', 10)
        self._rate = rospy.Rate(self._hz)
        self._tf = RobotLocalizationTf()
        self._kp = 0.1
        self._ki = 0.0
        self._kd = 0.0
        self._turned_on = False
        self._angle_diff = 0
        self._rate.sleep()

    def _current_pose_callback(self,data):
        self._curent_pose = data
        _, _, self._angle_diff,_ = self._tf.get_pose_diff(self._current_pose,self._goal_pose)

    def _goal_pose_callback(self,data):
        self._goal_pose = data

    def _on_callback(self,data):
        self._turned_on = data

    def set_pose_subscribers(self,current_pose_topic='current_pose',goal_pose_topic='goal_pose'):
        self._current_pose_sub = rospy.Subscriber(current_pose_topic, Pose, self._current_pose_callback)
        self._goal_pose_sub = rospy.Subscriber(goal_pose_topic, Pose, self._goal_pose_callback)
        self._switch = rospy.Subscriber('controller_on', Bool, self._on_callback)

    def set_constants(self,kp=0.1,ki=0,kd=0):
        self._kp = kp
        self._ki = ki
        self._kd = kd

    def _publish_vel(self,vel):
        self._pub_cmd.publish(vel)

    def _compute_velocity(self):
        angular = self._angle_diff*self._kp
        rospy.loginfo("angular: %f",angular)
        twist = Twist()
        twist.linear.x = 0.3 #hardcode
        twist.angular.z = angular
        return twist

    def _PID(self):
        twist = self._compute_velocity()
        self._publish_vel(twist)

    def run(self):
        while True:
            if(self._turned_on):
                self._PID()
            self._rate.sleep()

def main():
    rospy.init_node('pose_controller')
    app = PoseController()
    app.set_pose_subscribers()
    app.run()

main()