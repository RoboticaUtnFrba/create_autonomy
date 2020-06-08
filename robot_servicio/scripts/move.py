#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose


def listen_position():
    rospy.Subscriber("/create1/move_base_goals", Pose, move_to_goal)
    rospy.spin()

def move_to_goal(position_goal):
    rospy.logwarn("Receive goal")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = position_goal.position.x
    goal.target_pose.pose.position.y = position_goal.position.y
    goal.target_pose.pose.orientation = position_goal.orientation

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        #rospy.signal_shutdown("Action server not available!")
    else:
        if client.get_result():
            rospy.logwarn("Move success")
        else:
            rospy.logerr("Move not done")

if __name__ == '__main__':
    rospy.init_node('move')
    listen_position()