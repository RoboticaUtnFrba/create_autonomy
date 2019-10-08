#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math
from GTSys import GroundTruth
from RobotLocalizationTf import RobotLocalizationTf
from threading import Lock
from enum import Enum

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe



class RobotState(Enum):

    """
    Enumerative type for states of the robot.
    """

    STOP = 1
    MOVE_FORWARD = 2
    ROTATE = 3




class DrawSquare():
    
    """
    Class for implementing the drawing of a square in Gazebo.
    Probably it can be made more generic for drawing any kind of geometric figure.
    """

    ANGLE_THRESHOLD = 0.05
    ANGULAR_VEL = 0.3
    LINEAR_VEL = 0.7
    LENGTH_THRESHOLD = 0.05
    GOAL_DIST_THRESHOLD = 0.05

    _current_pose = Pose()
    _state = RobotState.STOP # Current  state of the robot
    _twist = Twist()
    _next_pose = Pose()
    _ground_truth = GroundTruth()
    _goal_angle = 0
    _currentGoalReached = False # Variable which determines if the next goal has to be given
    _goal_pose = Pose()
    _diff_angle = 0
    _length_diff = 999999
    _square_length = 2
    _tf = RobotLocalizationTf() # Transformations and computations for robot poses
    _goal_num = 0   #use iterables


    def __init__(self):
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._pub_controller_on = rospy.Publisher('controller_on', Bool, queue_size=10)
        self._pub_controller_goal = rospy.Publisher('goal_pose', Pose, queue_size=10)
        self._sub_control = rospy.Subscriber('cmd_vel_control', Twist, self._control_callback)
        self._lock = Lock()
        self._control_twist = Twist()
        self._controller_off()
        self._hz = rospy.get_param('~hz', 10)
        self._current_pose = self._ground_truth.get_pose()
        self._rate = rospy.Rate(self._hz)
        self._rate.sleep()

    def _publish(self,twist):
        self._pub_cmd.publish(twist)

    def _control_callback(self, data):
        self._lock.acquire()
        self._control_twist = data
        self._lock.release()

    def _controller_on(self):
        self._pub_controller_on.publish(True)
    
    def _controller_off(self):
        self._pub_controller_on.publish(False)

    def _update_controller_goal(self):
        self._pub_controller_goal.publish(self._goal_pose)

    def _rotate(self):
        rospy.loginfo("rotate")
        self._twist.angular.z = self.ANGULAR_VEL
        self._twist.linear.x = 0
        while(abs(self._diff_angle) > self.ANGLE_THRESHOLD):
            self._publish(self._twist)
            self._update_current_pose()
            (_, _, self._diff_angle, self._length_diff) = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
        self._stop()

    def _stop(self):
        rospy.loginfo("stop")
        self._twist.angular.z = 0
        self._twist.linear.x  = 0
        self._publish(self._twist)

    def _set_square_goal(self):

        """
        Function that sets the next goal for drawing a square
        """

        q = self._current_pose.orientation
        euler = efq((q.x,q.y,q.z,q.w))
        euler_yaw = self._tf.wrap_angle(euler[2] + math.pi/2.0)
        q = qfe(euler[0],euler[1],euler_yaw) 
        self._goal_pose.position.x = self._current_pose.position.x + self._square_length*math.cos(euler_yaw)
        self._goal_pose.position.y = self._current_pose.position.y + self._square_length*math.sin(euler_yaw)
        self._update_controller_goal()

    def _get_next_state(self):
        rospy.loginfo("Get next state")
        if(abs(self._diff_angle) > self.ANGLE_THRESHOLD):
            self._state = RobotState.ROTATE
        elif(self._length_diff > self.LENGTH_THRESHOLD):
            self._state = RobotState.MOVE_FORWARD
        else:
            self._state = RobotState.STOP
            self._currentGoalReached = True

    def _move_forward(self):
        rospy.loginfo("MF")
        self._controller_on()
        (_, _, self._diff_angle, self._length_diff) = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
        while(self._length_diff > self.LENGTH_THRESHOLD):
            self._publish(self._control_twist)
            self._update_current_pose()
            (_, _, _, self._length_diff) = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
            if(abs(self._diff_angle) > 1): # If something goes wrong and the angle gets very big, the function stops and the robot will start doing pure rotation
                break
        self._controller_off()
        self._stop()

    def _update_current_pose(self):
        self._current_pose = self._ground_truth.get_pose()

    def _goal_reached(self):

        """Function that determines if the robot has reached his goal.
        Returns:
            Boolean
        """

        rospy.loginfo("goal_reached")
        if(self._length_diff < self.GOAL_DIST_THRESHOLD ):
            self._currentGoalReached = True
            return True
        else:
            return False


    _actions = {RobotState.STOP:_stop , RobotState.MOVE_FORWARD:_move_forward, RobotState.ROTATE:_rotate}
    """Dictionary of functions for implementing more elegantly the calling of the functions according to the current robot state."""

    def _init_goal(self):
        self._goal_pose = Pose()


    def run(self):

        """
        Executes the global algorithm for drawing a square.
        """

        self._init_goal()
        rospy.loginfo("run")
        while True:
            self._update_current_pose()
            (_, _, self._diff_angle, self._length_diff) = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
            rospy.loginfo("diff angle: %f",self._diff_angle)
            rospy.loginfo("length diff: %f",self._length_diff)
            if(self._goal_reached()):
                self._set_square_goal()
                self._currentGoalReached = False
                (_, _, self._diff_angle, self._length_diff) = self._tf.get_pose_diff(self._current_pose, self._goal_pose)
            self._get_next_state()
            self._actions[self._state](self)
            self._rate.sleep()





def main():
    rospy.init_node('draw_square')
    app = DrawSquare()
    app.run()

main()