#! /usr/bin/env python

import roslib
roslib.load_manifest('ca_tools')
import rospy
import actionlib
import time

import ca_actions.msg
from follow_lines import FollowLines
from ground_truth import GroundTruth
from robot_localization_tf import RobotLocalizationTf


class LineFollowerServer:

    """Class for implementing the mission for the line-follower robot
    """

    # create messages that are used to publish feedback/result
    _feedback = ca_actions.msg.LineFollowerFeedback()
    _result = ca_actions.msg.LineFollowerResult()

    def __init__(self):
        self._action_name = name
        rospy.loginfo("1")
        self._as = actionlib.SimpleActionServer(self._action_name, ca_actions.msg.LineFollowerAction, execute_cb=self._execute_cb, auto_start = False)
        rospy.loginfo("2")
        self._as.start()
        self._gt = GroundTruth()
        self._time_elapsed = 0
        self._distance_travelled = 0
        self._last_pose = self._gt.get_pose()
        self._last_time = rospy.get_time()

    def _execute_cb(self, goal):
        
        """ Callback function for updating current data and checking whether the goal has been reached
        """

        rospy.loginfo("entered cb")
        # Begin assuming that the goal is not reached
        success = False
        # Get current state of the robot
        self._current_pose = self._gt.get_pose()
        self._current_time = rospy.get_time()
        # Compute time and distance variations from last callback until now
        _, _, _, pose_diff = RobotLocalizationTf.get_pose_diff(self._current_pose,self._last_pose)
        self._last_pose = self._current_pose
        time_diff = rospy.get_time() - self._last_pose
        # Update time and distance travelled
        self._feedback.distance += pose_diff
        self._feedback.time += time_diff
        self._feedback.position.x = self._current_pose
        # Check if goal has been reached
        success = _, _, _, pose_diff = RobotLocalizationTf.get_pose_diff(self._current_pose,goal)

        if success:
            self._result.result = self._feedback.feedback
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def run(self, goal):
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('line_follower_server')
    server = LineFollowerServer(rospy.get_name())
    rospy.spin()