#!/usr/bin/env python
import rospy
import threading
from ca_msgs.msg import Bumper
from geometry_msgs.msg import Twist, Vector3

class StateMachine(object):

  def __init__(self):
    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    self.goal_queue = []

  def rotate(self, ang_vel):
    self.move(0., ang_vel)

  def rotate_left(self, ang_vel):
    self.rotate(ang_vel)
  
  def rotate_right(self, ang_vel):
    self.rotate(-ang_vel)
  
  def set_goal(self, data):
    if data.is_left_pressed and data.is_right_pressed:
      self.goal_queue.append({'goal': self.move_backward, 'velocity': 0.1, 'duration': 3.})
    if data.is_left_pressed:
      self.goal_queue.append({'goal': self.move_backward, 'velocity': 0.1, 'duration': 1.5})
      self.goal_queue.append({'goal': self.rotate_right, 'velocity': 0.3, 'duration': 2.})
    elif data.is_right_pressed:
      self.goal_queue.append({'goal': self.move_backward, 'velocity': 0.1, 'duration': 1.5})
      self.goal_queue.append({'goal': self.rotate_left, 'velocity': 0.3, 'duration': 2.})
    else:
      self.goal_queue.append({'goal': self.move_straight, 'velocity': 0.2, 'duration': 0.})
  
  def stop(self):
    self.move(0., 0.)
  
  def close(self):
    self.stop()
    self.goal_queue = []
  
  def move(self, lin_vel, ang_vel):
    msg = Twist()
    msg.linear.x = lin_vel
    msg.angular.z = ang_vel
    self.pub.publish(msg)
  
  def move_straight(self, lin_vel):
    self.move(lin_vel, 0.)
  
  def move_backward(self, lin_vel):
    self.move_straight(-lin_vel)

  def run(self):
    if len(self.goal_queue) > 0:
      # Execute next goal
      goal = self.goal_queue.pop()
      end_time = rospy.Time.now().secs + goal.get('duration')
      while end_time > rospy.Time.now().secs:
        goal.get('goal')(goal.get('velocity'))
    else:
      # Move straight
      self.move_straight(0.2)

class RankingController():

  def __init__(self):
    rospy.init_node("ranking_controller", log_level=rospy.INFO)
    self.sub = rospy.Subscriber("bumper", Bumper, self.callback)
    self.state_machine = StateMachine()
    self.rate = rospy.Rate(10) # Hz
    rospy.on_shutdown(self.stop)
    threading.Thread(name="ranking_controller", target=self.run).start()
    rospy.spin()
  
  def callback(self, data):
    rospy.logdebug("{} {}".format(data.is_left_pressed, data.is_right_pressed))
    self.state_machine.set_goal(data)
  
  def stop(self):
    rospy.loginfo("Thread stopped.")
    self.state_machine.close()
  
  def run(self):
    rospy.loginfo("Thread started.")
    while not rospy.is_shutdown():
      self.state_machine.run()
      self.rate.sleep()

if __name__ == "__main__":
  rc = RankingController()
