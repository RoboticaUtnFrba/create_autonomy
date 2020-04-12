#!/usr/bin/env python
import rospy
import rostest
import unittest
import sys

from ca_msgs.msg import Bumper, Cliff

__author__ = 'Emiliano Borghi'

PKG = 'ca_gazebo'
NAME = 'publish'

class PublisherTests(unittest.TestCase):

  def __init__(self, *args):
    # Init ROS and params
    rospy.init_node(NAME, anonymous=True)
    self.timeout = rospy.get_param("~timeout_s", default=10.0)
    rospy.loginfo("Timeout set to {} seconds".format(self.timeout))
    # Call TestCase class
    super(PublisherTests, self).__init__(*args)

  def check(self, topic, msg_type, callback):
    self.message_triggered = False
    sub = rospy.Subscriber(topic, msg_type, callback)

    rate_hz = rospy.Rate(20)  # 20 Hz
    timeout_t = rospy.get_time() + self.timeout
    while (not rospy.is_shutdown()) and \
          (rospy.get_time() <= timeout_t) and \
          (not self.message_triggered):
      rate_hz.sleep()
    # Unregister subscriber
    sub.unregister()

    return self.message_triggered

  def test_bumper(self): # test names must start with 'test_'
    res = self.check('/create1/bumper', Bumper, self.bumper_cb)

    self.assertTrue(res, msg="Bumper message not received")

  def bumper_cb(self, msg):
    self.message_triggered |= (msg.is_left_pressed or msg.is_right_pressed)

  def test_cliff(self):
    res = self.check('/create1/cliff', Cliff, self.cliff_cb)

    self.assertFalse(res, msg="Cliff message not received")

  def cliff_cb(self, msg):
    self.message_triggered |= \
        (msg.is_cliff_left or msg.is_cliff_front_left or \
         msg.is_cliff_front_right or msg.is_cliff_right)

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, PublisherTests, sys.argv)
