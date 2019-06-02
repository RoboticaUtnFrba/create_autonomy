#!/usr/bin/env python

import os
import re
import rospy
import rospkg
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from tf.transformations import quaternion_from_euler

class RobotSpawner(object):
  
  SPAWN_URDF_TOPIC = '/gazebo/spawn_urdf_model'
  FILE_LOCATION = rospkg.RosPack().get_path("ca_description") + "/urdf/create_2.xacro"

  def __init__(self):
    rospy.init_node('robot_spawner')
    self.ns = rospy.get_namespace()
    # Get robot index
    try:
      index = re.findall('[0-9]+', self.ns)[0]
    except IndexError:
      index = 0
    i = index
    # Spawn URDF service client
    rospy.wait_for_service(RobotSpawner.SPAWN_URDF_TOPIC)
    try:
      spawn_urdf_model = rospy.ServiceProxy(RobotSpawner.SPAWN_URDF_TOPIC, SpawnModel)
      msg = SpawnModelRequest()
      msg.model_name = "irobot_create2.{}".format(i)
      inorder_flag = "--inorder" if self._get_ros_version() == "kinetic" else ""
      args = "visualize:=false"
      cmd = "rosrun xacro xacro {} {} {}".format(inorder_flag, RobotSpawner.FILE_LOCATION, args)
      p = os.popen(cmd)
      msg.model_xml = p.read()
      p.close()
      msg.robot_namespace = self.ns
      msg.initial_pose = Pose()
      msg.initial_pose.position.x = rospy.get_param("{}x".format(self.ns), 0.)
      msg.initial_pose.position.y = rospy.get_param("{}y".format(self.ns), 0.)
      q = quaternion_from_euler(0, 0, rospy.get_param("{}yaw".format(self.ns), 0.))
      msg.initial_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
      msg.reference_frame = "world"
      res = spawn_urdf_model(msg)
      print(res.status_message)
    except rospy.ServiceException, e:
      print("Could not spawn {}".format(msg.model_name))
      exit(1)
    
    print("{} spawned correctly".format(msg.model_name))
  
  def _get_ros_version(self):
    # Get ROS version
    p = os.popen("rosversion -d")
    ros_version = p.read
    p.close()
    return ros_version

def main():
  RobotSpawner()

if __name__ == "__main__":
  main()