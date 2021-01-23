#!/usr/bin/env python
import rospy
import unittest

import numpy as np

from sensor_msgs.msg import JointState

__author__ = 'Emiliano Borghi'

class StaticRobotTest(unittest.TestCase):

  def __init__(self, *args):
    self.timeout = rospy.get_param("static_test/timeout_s", default=10.0)
    rospy.loginfo("Timeout set to {} seconds".format(self.timeout))
    self.position_error = float(rospy.get_param("static_test/position_error", default=1e-6))
    # Call TestCase class
    super(StaticRobotTest, self).__init__(*args)

  def test_static_robot(self):
    self.initial_position = None
    self.final_position = None
    sub = rospy.Subscriber('/create1/joint_states', JointState, self.joint_state_cb)

    rate_hz = rospy.Rate(20)  # 20 Hz
    timeout_t = rospy.get_time() + self.timeout
    while (not rospy.is_shutdown()) and \
          (rospy.get_time() <= timeout_t):
      rate_hz.sleep()
    # Unregister subscriber
    sub.unregister()

    # Checking that wheels have not been moved during the specified time
    diff = np.abs(np.diff([self.final_position, self.initial_position]))
    self.assertLessEqual(
      diff.item(0),
      self.position_error,
      msg="The robot is not static. Left wheel has moved {} rad".format(str(diff.item(0))))
    self.assertLessEqual(
      diff.item(1),
      self.position_error,
      msg="The robot is not static. Right wheel has moved {} rad".format(str(diff.item(1))))

  def joint_state_cb(self, msg):
    if self.initial_position is None:
      self.initial_position = msg.position
    else:
       self.final_position = msg.position
