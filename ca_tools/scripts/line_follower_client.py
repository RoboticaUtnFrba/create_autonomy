#! /usr/bin/env python

import roslib
roslib.load_manifest('ca_tools')
import rospy
import actionlib

from ca_actions.msg import LineFollowerAction, LineFollowerFeedback, LineFollowerResult, LineFollowerGoal

def line_follower_client():

    client = actionlib.SimpleActionClient('line_follower', LineFollowerAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = LineFollowerGoal()
    goal.goal.time = 2
    goal.goal.distance = 3

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Gets the result of executing the action
    return client.get_result()


if __name__ == '__main__':

    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS. 
    rospy.init_node('line_follower_client')
    result = line_follower_client()


