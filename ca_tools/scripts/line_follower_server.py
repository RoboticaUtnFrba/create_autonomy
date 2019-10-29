#! /usr/bin/env python

import roslib
roslib.load_manifest('ca_tools')
import rospy
import actionlib
import math

from ca_actions.msg import LineFollowerAction, LineFollowerFeedback, LineFollowerResult
from ground_truth import GroundTruth
from robot_localization_tf import RobotLocalizationTf
from std_msgs.msg import Bool
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
        self._as.start()
        self._gt = GroundTruth()
        self._time_elapsed = 0
        self._distance_travelled = 0
        self._r = rospy.Rate(2)

    def _execute_cb(self, goal):
        
        """ Callback function for updating current data and checking whether the goal has been reached
        """

        success = True
        self._last_pose = self._gt.get_pose()
        self._last_time = rospy.Time.now()

        while(self._feedback.feedback.time < goal.goal.time and self._feedback.feedback.distance < goal.goal.distance):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            # Get current state of the robot
            self._current_pose = self._gt.get_pose()
            self._current_time = rospy.Time.now()
            # Compute time and distance variations from last loop until now
            diff_x = (self._current_pose.position.x - self._last_pose.position.x)
            diff_y = (self._current_pose.position.y - self._last_pose.position.y)
            pose_diff = math.hypot(diff_x, diff_y)
            #pose_diff = RobotLocalizationTf.get_position_diff(self._current_pose.position,self._last_pose.position)
            self._last_pose = self._current_pose
            time_diff = (self._current_time - self._last_time).to_sec() 
            self._last_time = self._current_time
            # Update time and distance travelled
            self._feedback.feedback.distance += pose_diff
            self._feedback.feedback.time += time_diff
            self._feedback.feedback.position = self._last_pose.position
            self._as.publish_feedback(self._feedback)
            # Check if goal has been reached
            pose_diff = RobotLocalizationTf.get_position_diff(self._current_pose.position,goal.goal.position)
            self._result.result = pose_diff < self._goal_diff_threshold
            self._r.sleep()

        if success:
            self._as.set_succeeded(self._result)




if __name__ == '__main__':
    rospy.init_node('line_follower_server')
    server = LineFollowerServer()
    rospy.spin()