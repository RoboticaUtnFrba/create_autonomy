#!/usr/bin/env python

from MPU6050 import MPU6050

import rospy
import numpy as np

class MPU6050Calibration(object):

    def __init__(self):
        rospy.init_node('mpu6050_calibration', log_level=rospy.INFO)
        rospy.on_shutdown(self.__shutdown_cb)

        # The offsets are different for each device and should be changed
        # accordingly using a calibration procedure
        x_accel_offset = int(rospy.get_param('~x_acc', 0))
        y_accel_offset = int(rospy.get_param('~y_acc', 0))
        z_accel_offset = int(rospy.get_param('~z_acc', 0))
        x_gyro_offset = int(rospy.get_param('~x_gyro', 0))
        y_gyro_offset = int(rospy.get_param('~y_gyro', 0))
        z_gyro_offset = int(rospy.get_param('~z_gyro', 0))

        enable_debug_output = int(rospy.get_param('~debug', True))

        i2c_bus = int(rospy.get_param('~i2c_bus', 1))
        device_address = int(rospy.get_param('~device_addr', 0x68))

        # Calibration parameters
        self.buffer_size = int(rospy.get_param('~buffer_size', 1000))
        self.accel_deadzone = int(rospy.get_param('~accel_deadzone', 8))
        self.gyro_deadzone = int(rospy.get_param('~gyro_deadzone', 1))

        self.mean_rot = np.zeros(3)
        self.mean_acc = np.zeros(3)

        self.mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
                      z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
                      enable_debug_output)

    def __shutdown_cb(self):
        rospy.loginfo("Shutting down MPU6050")
        self.mpu.reset()

    def __mean_sensors(self):
        # Variables to keep track of the readings
        ROT = np.zeros(3)
        ACCEL = np.zeros(3)
        # Number of samples that will be discarded at the beginning
        n_discard = 100
        for measure in xrange(self.buffer_size + n_discard):
            # Read raw accel and gyro measurements
            rot = self.mpu.get_rotation()
            acc = self.mpu.get_acceleration()
            # Discard first "n_discard" measurement
            if measure > n_discard:
                ROT += rot
                ACCEL += acc
            # Sleep during 2 ms to avoid duplicated measurements
            rospy.sleep(2/1000.)
        # Compute average
        self.mean_rot = ROT / self.buffer_size
        self.mean_acc = ACCEL / self.buffer_size

    def __calibration(self):
        self.a_x_offset = -self.mean_acc[0] / 8.
        self.a_y_offset = -self.mean_acc[1] / 8.
        self.a_z_offset = (16384 - self.mean_acc[2]) / 8.
        self.g_offset = -self.mean_rot / 4.

        params_calibrated = [False] * 6

        while all(params_calibrated):
            self.mpu.set_x_accel_offset(self.a_x_offset)
            self.mpu.set_y_accel_offset(self.a_y_offset)
            self.mpu.set_z_accel_offset(self.a_z_offset)
            self.mpu.set_x_gyro_offset(self.g_offset[0])
            self.mpu.set_y_gyro_offset(self.g_offset[1])
            self.mpu.set_z_gyro_offset(self.g_offset[2])

            self.__mean_sensors()
            rospy.loginfo("...")

            if abs(self.mean_acc[0]) <= accel_deadzone:
                params_calibrated[0] = True
            else:
                self.a_x_offset = (self.a_x_offset - self.mean_acc[0]) / accel_deadzone

            if abs(self.mean_acc[1]) <= accel_deadzone:
                params_calibrated[1] = True
            else:
                self.a_y_offset = (self.a_y_offset - self.mean_acc[1]) / accel_deadzone

            if abs(16384 - self.mean_acc[2]) <= accel_deadzone:
                params_calibrated[2] = True
            else:
                self.a_z_offset = (self.a_z_offset - self.mean_acc[2]) / accel_deadzone

            if abs(self.mean_rot[0]) <= gyro_deadzone:
                params_calibrated[3] = True
            else:
                self.g_offset[0] = (self.g_offset[0] - self.mean_rot[0]) / gyro_deadzone

            if abs(self.mean_rot[1]) <= gyro_deadzone:
                params_calibrated[4] = True
            else:
                self.g_offset[1] = (self.g_offset[1] - self.mean_rot[1]) / gyro_deadzone

            if abs(self.mean_rot[2]) <= gyro_deadzone:
                params_calibrated[5] = True
            else:
                self.g_offset[2] = (self.g_offset[2] - self.mean_rot[2]) / gyro_deadzone

    def calibrate(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Reading sensor values...")
            self.__mean_sensors()
            rospy.sleep(1)

            rospy.loginfo("Calculating offsets...")
            self.__calibration()
            rospy.sleep(1)

            self.__mean_sensors()
            rospy.loginfo("FINISHED!")
            rospy.loginfo("Sensor readings with offset:")
            rospy.loginfo(self.mean_acc[0])
            rospy.loginfo(self.mean_acc[1])
            rospy.loginfo(self.mean_acc[2])
            rospy.loginfo(self.mean_rot[0])
            rospy.loginfo(self.mean_rot[1])
            rospy.loginfo(self.mean_rot[2])
            rospy.loginfo("Your offsets:")
            rospy.logwarn("x_acc: {}".format(self.a_x_offset))
            rospy.logwarn("y_acc: {}".format(self.a_y_offset))
            rospy.logwarn("z_acc: {}".format(self.a_z_offset))
            rospy.logwarn("x_gyro: {}".format(self.g_offset[0]))
            rospy.logwarn("y_gyro: {}".format(self.g_offset[1]))
            rospy.logwarn("z_gyro: {}".format(self.g_offset[2]))

            rospy.loginfo("Replace these values into calibrated.yaml")

            exit()

if __name__ == '__main__':
    calib = MPU6050Calibration()
    calib.calibrate()
