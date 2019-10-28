#! /usr/bin/env python


import roslib
roslib.load_manifest('ca_tools')
import rospy
import actionlib

from ca_actions.msg import LineFollowerAction, LineFollowerGoal

def line_follower_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('line_follower_client', LineFollowerAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = LineFollowerGoal()
    goal.goal.position.x = 2
    goal.goal.position.y = -4
    goal.goal.position.z = 0

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('line_follower_client')
    result = line_follower_client()
    print(result)