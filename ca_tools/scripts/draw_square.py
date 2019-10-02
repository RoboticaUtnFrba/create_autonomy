#! /usr/bin/env python
# -*- coding: utf-8 -*-


import math
from GTSys import GTSys

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe



class RobotState():

    STATES = [
    "MOVE_FORWARD",
    "ROTATE",
    "STOP"
    ]

    _current_state = None

    def __init__(self):
        self._current_state = "STOP"

    def SetState(self,state):
        "ToDo: assert a valid state"
        self._current_state = state

    def GetState(self):
        return self._current_state





class DrawSquare():

    ANGLE_THRESHOLD = 0.05
    ANGULAR_VEL = 0.3
    LINEAR_VEL = 0.5
    LENGTH_THRESHOLD = 0.05
    GOAL_DIST_THRESHOLD = 0.1

    _current_pose = Pose()
    _state = RobotState()
    _twist = Twist()
    _next_pose = Pose()
    _ground_truth = GTSys()
    _goal_angle = 0
    _currentGoalReached = False
    _goal_pose = Pose()
    _diff_angle = 0
    _length_diff = 0
    _square_length = 2


    def __init__(self):
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._hz = rospy.get_param('~hz', 10)
        self._current_pose = self._ground_truth.get_pose()
        self._rate = rospy.Rate(self._hz)
        self._rate.sleep()


    def _wrap_angle(self,angle): #ToDo: make beautiful this horrible function
        while (angle<0):
            angle+=2.0*math.pi
        while (angle>2*math.pi):
            angle-=2.0*math.pi
        return angle


    def _publish(self):
        self._pub_cmd.publish(self._twist)

    def _rotate(self):
        rospy.loginfo("rotate")
        self._twist.angular.z = self.ANGULAR_VEL
        self._twist.linear.x = 0

        while(self._diff_angle > self.ANGLE_THRESHOLD):
            self._publish()
            self._update_current_pose()
            self._get_pose_diff()
        self._stop()


    def _stop(self):
        rospy.loginfo("stop")
        self._twist.angular.z = 0
        self._twist.linear.x  = 0
        self._publish()

    def _get_pose_diff(self):
        q = self._current_pose.orientation
        euler = efq((q.x,q.y,q.z,q.w))
        self._current_angle = self._wrap_angle(euler[2])

        diff_x = (self._goal_pose.position.x - self._current_pose.position.x)
        diff_y = (self._goal_pose.position.y - self._current_pose.position.y)

        self._goal_angle = self._wrap_angle(math.atan2(diff_y,diff_x))
        self._diff_angle = self._wrap_angle(self._goal_angle - self._current_angle)
        self._length_diff = math.sqrt(diff_x*diff_x + diff_y*diff_y)

    def _set_square_goal(self):
            q = self._current_pose.orientation
            euler = efq((q.x,q.y,q.z,q.w))
            euler_yaw = self._wrap_angle(euler[2] + math.pi/2.0)
            q = qfe(euler[0],euler[1],euler_yaw) 
            self._goal_pose.position.x = self._current_pose.position.x + self._square_length*math.cos(euler_yaw)
            self._goal_pose.position.y = self._current_pose.position.y + self._square_length*math.sin(euler_yaw)
            return 

    def _get_next_state(self):
        rospy.loginfo("Get next state")
        if(self._diff_angle > self.ANGLE_THRESHOLD):
            self._state.SetState("ROTATE")
        elif(self._length_diff > self.LENGTH_THRESHOLD):
            self._state.SetState("MOVE_FORWARD")
        else:
            self._state.SetState("STOP")
            self._currentGoalReached = True


    def _move_forward(self):
        rospy.loginfo("MF")
        t0 = rospy.get_time()
        distance_travelled = 0
        self._twist.linear.x = self.LINEAR_VEL
        self._twist.angular.z = 0
        while(abs(distance_travelled - self._length_diff) > self.LENGTH_THRESHOLD):
            self._publish()
            t1 = rospy.get_time()
            distance_travelled = (self._twist.linear.x)*(t1-t0)
        self._length_diff -= distance_travelled
        self._stop()


    def _update_current_pose(self):
        self._current_pose = self._ground_truth.get_pose() #Could be better implemented using callbacks

    def _goal_reached(self):
        rospy.loginfo("goal_reached")
        if(self._length_diff < self.GOAL_DIST_THRESHOLD ):
            self._currentGoalReached = True
            return True
        else:
            return False

    
    _actions = {'MOVE_FORWARD':_move_forward ,'STOP':_stop ,'ROTATE':_rotate}

    def run(self):
        rospy.loginfo("run")
        aux = Pose()
        aux.position.x = -1
        aux.position.y = -2
        aux.position.z = 0
        self._goal_pose = aux

        while True:
            self._update_current_pose()
            self._get_pose_diff()

            if(self._goal_reached()):
                self._set_square_goal()
                self._currentGoalReached = False
                self._get_pose_diff()
            
            self._get_next_state()
            self._actions[self._state.GetState()](self)

            rospy.loginfo("First goal: %r",self._currentGoalReached)
            rospy.loginfo("diff angle: %f",self._diff_angle)
            rospy.loginfo("diff length :%f",self._length_diff)
            rospy.loginfo("goal x: %f", self._goal_pose.position.x)
            rospy.loginfo("goal y: %f", self._goal_pose.position.y)
            self._rate.sleep()





def main():
    rospy.init_node('draw_square')
    app = DrawSquare()
    app.run()

main()