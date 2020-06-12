#!/usr/bin/env python
import rospy
import rostest
import unittest
import sys
import tf2_ros

__author__ = 'Emiliano Borghi'

PKG = 'ca_gazebo'
NAME = 'tf_checker'

class TfCheckerTests(unittest.TestCase):

  def __init__(self, *args):
    # Call TestCase class
    super(TfCheckerTests, self).__init__(*args)

  def setUp(self):
    # Init ROS and params
    rospy.init_node(NAME, anonymous=True)
    # Setup the tf listener
    self.buffer = tf2_ros.Buffer()
    self.tl = tf2_ros.TransformListener(self.buffer)

  def test_robot_description_param(self):
    robot_description_param = rospy.get_param("create1/robot_description", False)
    self.assertNotEqual(robot_description_param, False)

  def check_tree(self, parent, child):
    try:
      self.assertRaises(
        self.buffer.lookup_transform(
          'create1_tf/base_link', 'create1_tf/base_footprint',
          rospy.Time(), rospy.Duration(5)
        )
      )
    except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      self.assertFalse(True, "Test timed out waiting for the transform to be broadcast.")

  def test_frame_exists(self):
    # Testing tree
    self.tf_tree = rospy.get_param("tf_test")
    # Check correct tree
    for parent in self.tf_tree:
      for child in self.tf_tree[parent]:
        rospy.loginfo("Checking Tf {} --> {}".format(parent, child))
        self.check_tree('create1_tf/' + parent, 'create1_tf' + child)

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TfCheckerTests, sys.argv)
