#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class ImuConverter(object):

    def __init__(self):
        rospy.init_node("imu_converter", log_level=rospy.INFO)
        self.sub = rospy.Subscriber("/imu/data", Imu, self.__imu_cb__)
        rospy.spin()
    
    def __imu_cb__(self, msg):
        q = msg.orientation
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        rospy.loginfo("{}".format(math.degrees(euler[2])))

if __name__ == "__main__":
    ImuConverter()