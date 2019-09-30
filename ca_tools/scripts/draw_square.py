#! /usr/bin/env python
# -*- coding: utf-8 -*-


import curses
import math

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose


class RobotState():

    STATES = [
    "MOVE_FORWARD",
    "STOP_FORWARD",
    "ROTATE",
    "STOP_ROTATE"
    ]

    def __init(self):
        current_state = "STOP"

    def SetState(self,state):
        "assert state > 0 and max_velocity > 0 and num_steps > 0"
        current_state = state

    def GetState(self):
        return current_state



class DrawSquare():


    _current_pose = Pose()
    _linear = None
    _angular = None
    _state = RobotState()
    _twist = Twist()
    next_pose = Pose()
    next_pose.position.x=3.0
    init_t = 0
    final_t = 0



    def __init__(self):
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._sub_cmd = rospy.Subscriber('cmd_vel', Twist, self._callback)

        self._hz = rospy.get_param('~hz', 10)

    def _publish(self,twist):
        self._pub_cmd.publish(twist)

    def _callback(self,data):
        self._twist=data

    def _rotate(self):
        self._twist.angular.z = 0.7
        self._publish(twist)

    def _stop(self):
        self._twist.angular.z = 0
        self._twist.linear.x  = 0
        self._publish(self._twist)
        self.final_t = rospy.get_time()

    def _getTwist(self,goal_pose):
        diff_x = (goal_pose.position.x - self._current_pose.position.x)
        diff_y = (goal_pose.position.y - self._current_pose.position.y)
        self._twist.angular.z = math.atan2(diff_y,diff_x)
        self._twist.linear.x  = 0.3

    def _advance(self):
        self._getTwist(self.next_pose)
        self.init_t = rospy.get_time()
        self._publish(self._twist)

    def _getState(self):
        return state.GetState()

    def _updateCurrentPose(self):
        t_elapsed = self.final_t-self.init_t
        self._current_pose.position.x += math.sin(self._twist.angular.z)*(self._twist.linear.x)*(t_elapsed)
        self._current_pose.position.y += math.cos(self._twist.angular.z)*(self._twist.linear.x)*(t_elapsed)

    def run(self):
        rate = rospy.Rate(self._hz)
        while True:
            self.next_pose.position.y=0
            self.next_pose.position.x=0
            self._advance()
            self._updateCurrentPose()
            rate.sleep()
            rate.sleep()
            rate.sleep()
            rate.sleep()
            self._stop()
            self.next_pose.position.y=3
            self.next_pose.position.x=3
            self._advance()


def main(stdscr):
    rospy.init_node('draw_square')
    app = DrawSquare()
    app.run()
  
if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass