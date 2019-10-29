#! /usr/bin/env python

import roslib
roslib.load_manifest('ca_tools')
import rospy
import actionlib

from ca_actions.msg import LineFollowerAction, LineFollowerFeedback, LineFollowerResult
from ground_truth import GroundTruth
from robot_localization_tf import RobotLocalizationTf
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose


class LineFollowerServer:

    """Class for implementing the mission for the line-follower robot
    """

    # create messages that are used to publish feedback/result
    _feedback = LineFollowerFeedback()
    _result = LineFollowerResult()
    _goal_diff_threshold = 0.1

    def __init__(self):
        self._as = actionlib.SimpleActionServer('line_follower', LineFollowerAction, self._execute_cb, False)
        self._move_robot_pub = rospy.Publisher('/line_follower_is_on', Bool, queue_size=10)
        self._as.start()
        self._gt = GroundTruth()
        self._time_elapsed = 0
        self._distance_travelled = 0
        self._last_pose = self._gt.get_pose()
        self._last_time = rospy.get_time()
        self._r = rospy.Rate(1)

    def _execute_cb(self, goal):
        
        """ Callback function for updating current data and checking whether the goal has been reached
        """

        move_robot = Bool()
        move_robot.data = True
        self._move_robot_pub.publish(move_robot)

        # Begin assuming that the goal is not reached
        success = False

        while(self._time_elapsed < goal.goal.time and self._distance_travelled < goal.goal.distance):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                move_robot.data = False
                self._move_robot_pub.publish(move_robot)
                break

            # Get current state of the robot
            self._current_pose = self._gt.get_pose()
            self._current_time = rospy.get_time()
            # Compute time and distance variations from last callback until now
            pose_diff = RobotLocalizationTf.get_position_diff(self._current_pose.position,self._last_pose.position)
            self._last_pose = self._current_pose
            time_diff = rospy.get_time() - self._last_time
            # Update time and distance travelled
            self._feedback.feedback.distance += pose_diff
            self._feedback.feedback.time += time_diff
            self._feedback.feedback.position = self._last_pose.position
            self._as.publish_feedback(self._feedback)

            # Check if goal has been reached
            pose_diff = RobotLocalizationTf.get_position_diff(self._current_pose.position,goal.goal.position)
            success = pose_diff < self._goal_diff_threshold #hardcode


        if success:
            self._result.result = self._feedback.feedback
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            move_robot.data = False
            self._move_robot_pub.publish(move_robot)

        self._r.sleep()


if __name__ == '__main__':
    rospy.init_node('line_follower_server')
    server = LineFollowerServer()
    rospy.spin()