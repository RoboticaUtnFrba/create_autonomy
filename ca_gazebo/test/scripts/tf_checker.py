#!/usr/bin/env python
import rospy
import unittest
import tf2_ros

__author__ = 'Emiliano Borghi'

class TfCheckerTests(unittest.TestCase):

  def __init__(self, *args):
    # Call TestCase class
    super(TfCheckerTests, self).__init__(*args)

  def setUp(self):
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
          'create1/base_link', 'create1/base_footprint',
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
        self.check_tree('create1/' + parent, 'create1' + child)
