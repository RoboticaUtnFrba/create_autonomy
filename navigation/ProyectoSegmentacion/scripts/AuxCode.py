
#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist,Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf2_geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
def movebase_client():
  # Publisher to manually control the robot (e.g. to stop it)
  #cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
 # Create an action client called "move_base" with action definition file "MoveBaseAction"
    rospy.Subscriber("/create1/odom", Odometry, self.callback)
    pub = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(10) # 10hz
  client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
  print("actionlib inicializado\n")
 # Waits until the action server has started up and started listening for goals.
  client.wait_for_server(rospy.Duration(5.0))
  print("sale del wait for server\n")
     
 # Creates a new goal with the MoveBaseGoal constructor
  goal = MoveBaseGoal()
  print("Llama a move_base goal\n")
  goal.target_pose.header.frame_id = "map"
  goal.target_pose.header.stamp = rospy.Time.now()
 # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
  goal.target_pose.pose.position.x = 0.0
  goal.target_pose.pose.position.y = 0.0
 # No rotation of the mobile base frame w.r.t. map frame
  goal.target_pose.pose.orientation.w = 1.0

 # Sends the goal to the action server.
  client.send_goal(goal)
  print("manda el goal\n")
 # Waits for the server to finish performing the action.
  wait = client.wait_for_result()
  print("sale del wait for result\n")
     
 # If the result doesn't arrive, assume the Server is not available
  if not wait:
      client.cancel_goal()
      rospy.logerr("Action server not available!")
      rospy.signal_shutdown("Action server not available!")
  else:
  # Result of executing the action
      return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
  #rospy.logerr("Empezando a navegar")
  try:
      print("inicia nodo\n")
     # Initializes a rospy node to let the SimpleActionClient publish and subscribe
      rospy.init_node('mi_nodo')
      result = movebase_client()
      if result:
          rospy.loginfo("Goal execution done!")
  except rospy.ROSInterruptException:
      rospy.loginfo("Navigation test finished.")
