#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler


class send_goals():
    def __init__(self):
        self.pi = 3.1416

        sec_1_table_1 = Pose()
        self.load_position(sec_1_table_1, 0, 4.5, 180)
        sec_1_table_2 = Pose()
        self.load_position(sec_1_table_2, -2, 4.5, 180)
        sec_1_table_3 = Pose()
        self.load_position(sec_1_table_3, -3.5, 4.5, 180)

        sec_2_table_left = Pose()
        self.load_position(sec_2_table_left, 5, 0, 14)
        sec_2_table_right = Pose()
        self.load_position(sec_2_table_right, -2.5, 0, 14)

        sec_3_table_small_1 = Pose()
        self.load_position(sec_3_table_small_1, 4.8, -1, -90)
        sec_3_table_small_2 = Pose()
        self.load_position(sec_3_table_small_2, 4.8, -5, -90)
        sec_3_table_big = Pose()
        self.load_position(sec_3_table_big, 4.8, -2.5, -90)
        
        self.goals = rospy.Publisher('/create1/move_base_goals', Pose, queue_size=10)

        rospy.sleep(1) # wait to be ready for publish
        r = rospy.Rate(0.1) # 0.1hz

        while not rospy.is_shutdown():
            self.goals.publish(sec_1_table_1)
            r.sleep()
            self.goals.publish(sec_1_table_2)
            r.sleep()
            self.goals.publish(sec_1_table_3)
            r.sleep()
            self.goals.publish(sec_2_table_left)
            r.sleep()
            self.goals.publish(sec_2_table_right)
            r.sleep()
            self.goals.publish(sec_3_table_small_1)
            r.sleep()
            self.goals.publish(sec_3_table_small_2)
            r.sleep()
            self.goals.publish(sec_3_table_big)
            r.sleep()


    def load_position(self, goal_pose, x, y, angle_raw):
        q = quaternion_from_euler(0, 0, self.pi * angle_raw / 180)
        goal_pose.position.x = x
        goal_pose.position.y = y
        goal_pose.orientation.x = q[0]
        goal_pose.orientation.y = q[1]
        goal_pose.orientation.z = q[2]
        goal_pose.orientation.w = q[3]


if __name__ == '__main__':
    rospy.init_node('send_goals')
    send_goals()