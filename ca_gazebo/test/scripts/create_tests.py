#! /usr/bin/env python
import unittest
import rospy
import rostest

from tf_checker import TfCheckerTests
from static import StaticRobotTest
from hztest import HzTest

class CreateTests(unittest.TestSuite):
  def __init__(self):
    rospy.init_node('create_tests')
    super(CreateTests, self).__init__()

    tests = [
      TfCheckerTests('test_robot_description_param'),
      TfCheckerTests('test_frame_exists'),
      StaticRobotTest('test_static_robot'),
      HzTest('test_hz', 'bumper'),
      HzTest('test_hz', 'cliff'),
      HzTest('test_hz', 'cliff_front_left'),
      HzTest('test_hz', 'cliff_front_right'),
      HzTest('test_hz', 'cliff_side_left'),
      HzTest('test_hz', 'cliff_side_right'),
      HzTest('test_hz', 'gts'),
      HzTest('test_hz', 'imu_data'),
      HzTest('test_hz', 'joint_states'),
      HzTest('test_hz', 'odom'),
      HzTest('test_hz', 'rplidar_scan'),
      HzTest('test_hz', 'virtual_wall'),
      HzTest('test_hz', 'wall'),
    ]
    self.addTests(tests)


if __name__ == '__main__':
  rostest.rosrun('create_gazebo', 'create_tests', 'create_tests.CreateTests')
