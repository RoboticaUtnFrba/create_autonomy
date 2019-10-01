#! /usr/bin/env python
# -*- coding: utf-8 -*-


import curses
import math

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe

ANGLE_THRESHOLD = 0.05
QUEUE_SIZE = 10
ANGULAR_VEL = 0.5
LINEAR_VEL = 0.5
LENGTH_THRESHOLD = 0.1
GOAL_DIST_THRESHOLD = 0.1


class RobotState():

    STATES = [
    "MOVE_FORWARD",
    "STOP_FORWARD",
    "ROTATE",
    "STOP_ROTATE"
    ]

    def __init__(self):
        current_state = "STOP"

    def SetState(self,state):
        "assert a valid state"
        current_state = state

    def GetState(self):
        return current_state


class GTSys():

    _pose = Pose()

    def __init__(self):
        self._sub_gts=rospy.Subscriber('gts', Odometry, self._callback)

    def _callback(self,data):
        self._pose = data.pose

    def GetPose(self):
        return self._pose



class DrawSquare():


    _current_pose = PoseWithCovariance()
    _linear = None
    _angular = None
    _state = RobotState()
    _twist = Twist()
    _next_pose = Pose()
    _ground_truth = GTSys()
    _goal_angle = 0
    _firstGoalReached = False
    _DEFAULT_LENGTH = 1
    _goal_pose = Pose()


    def __init__(self):
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist, QUEUE_SIZE)
        self._sub_cmd = rospy.Subscriber('cmd_vel', Twist, self._callback)
        self._hz = rospy.get_param('~hz', 10) #hardcode
        self._current_pose = self._ground_truth.GetPose()
        self._rate = rospy.Rate(self._hz)
        self._rate.sleep()


    def _wrapAngle(self,angle): #ToDo: make beautiful this horrible function
        while (angle<0):
            angle+=2.0*math.pi
        while (angle>2*math.pi):
            angle-=2.0*math.pi
        return angle


    def _publish(self):
        self._pub_cmd.publish(self._twist)

    def _callback(self,data):
        self._twist=data

    def _rotate(self):
        rospy.loginfo("rotate")
        self._twist.angular.z = ANGULAR_VEL
        self._twist.linear.x = 0

        while(self._diff_angle > ANGLE_THRESHOLD):
            self._publish()
            self._updateCurrentPose()
            self._getAngle()
        self._stop()


    def _stop(self):
        rospy.loginfo("stop")
        self._twist.angular.z = 0
        self._twist.linear.x  = 0
        self._publish()

    def _getAngle(self):
        rospy.loginfo("Get angle")
        q = self._current_pose.pose.orientation
        euler = efq((q.x,q.y,q.z,q.w))
        self._current_angle = self._wrapAngle(euler[2])

        diff_x = (self._goal_pose.position.x - self._current_pose.pose.position.x)
        diff_y = (self._goal_pose.position.y - self._current_pose.pose.position.y)

        self._goal_angle = self._wrapAngle(math.atan2(diff_y,diff_x))
        self._diff_angle = self._wrapAngle(self._goal_angle - self._current_angle)
        self._length_diff = math.sqrt(diff_x*diff_x + diff_y*diff_y)

    def _setSquareGoal(self):
            q = self._current_pose.pose.orientation
            euler = efq((q.x,q.y,q.z,q.w))
            euler_yaw = self._wrapAngle(euler[2] + math.pi/2.0))
            q = qfe(euler[0],euler[1],euler_yaw) 
            self._goal_pose.position.x = self._current_pose.pose.position.x + math.cos(euler_yaw)
            self._goal_pose.position.y = self._current_pose.pose.position.y + math.sin(euler_yaw)
            return 

        


    def _moveForward(self):
        rospy.loginfo("MF")
        t0 = rospy.get_time()
        distance_travelled = 0
        self._twist.linear.x = LINEAR_VEL
        self._twist.angular.z = 0
        while(abs(distance_travelled - self._length_diff) > LENGTH_THRESHOLD):
            self._publish()
            t1 = rospy.get_time()
            distance_travelled = (self._twist.linear.x)*(t1-t0)
        self._length_diff -= distance_travelled
        self._stop()


    def _updateCurrentPose(self):
        self._current_pose = self._ground_truth.GetPose() #Could be better implemented using callbacks

    def _goalReached(self):
        rospy.loginfo("goal_reached")
        if(self._length_diff < GOAL_DIST_THRESHOLD ):
            self._firstGoalReached = True
            return True
        else:
            return False

    
    _actions = {'MOVE_FORWARD':_moveForward ,'STOP':_stop ,'ROTATE':_rotate}

    def run(self):
        rospy.loginfo("run")
        aux = Pose()
        aux.position.x = -1
        aux.position.y = -2
        aux.position.z = 0
        self._goal_pose = aux

        while True:
            self._updateCurrentPose()

            if(self._firstGoalReached):
                self._setSquareGoal()
                self._firstGoalReached=False
            
            self._getAngle()

            if(self._goalReached()):
                self._stop()
            elif(self._diff_angle < ANGLE_THRESHOLD):
                self._moveForward()
            else:
                self._rotate()

            rospy.loginfo("First goal: %r",self._firstGoalReached)
            rospy.loginfo("diff angle: %f",self._diff_angle)
            rospy.loginfo("diff length :%f",self._length_diff)
            rospy.loginfo("goal x: %f", self._goal_pose.position.x)
            rospy.loginfo("goal y: %f", self._goal_pose.position.y)
            for i in range(0,10):
                self._rate.sleep()












def main(stdscr):
    rospy.init_node('draw_square')
    app = DrawSquare()
    app.run()
    '''aux = Pose()
    aux.position.x = 1
    aux.position.y = 0.1
    aux.position.z = 0
    app._getAngle(aux)

    for i in range(0,30):
        app._rate.sleep()'''

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass