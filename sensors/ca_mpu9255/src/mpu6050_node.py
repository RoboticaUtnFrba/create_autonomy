#!/usr/bin/env python
__author__ = 'Emiliano Borghi'
"""
MPU6050 Python I2C Class - MPU6050 example usage
Copyright (c) 2015 Geir Istad
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from MPU6050 import MPU6050

import rospy
from sensor_msgs.msg import Imu

lin_acc_rescale = 9.80665 / 16384.

DEFAULT_SAMPLE_RATE_HZ = 10
MPU_FRAMEID = "imu_link"

rospy.init_node('mpu6050')
imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)

sample_rate = int(rospy.get_param('~frequency', DEFAULT_SAMPLE_RATE_HZ))
frame_id = str(rospy.get_param('~frame_id', MPU_FRAMEID))

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

r = rospy.Rate(sample_rate)

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

def shutdown_cb():
    rospy.loginfo("Shutting down MPU6050")
    mpu.reset()
rospy.on_shutdown(shutdown_cb)

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
rospy.loginfo(hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
rospy.loginfo(packet_size)
FIFO_count = mpu.get_FIFO_count()
rospy.loginfo(FIFO_count)

FIFO_buffer = [0]*64

while not rospy.is_shutdown():
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()

    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
        rospy.logwarn('overflow!')
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)

        quat = mpu.DMP_get_quaternion(FIFO_buffer)
        accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        lin_accel = mpu.DMP_get_linear_accel(accel, grav)
        # roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        gyro = mpu.get_rotation()

        now = rospy.Time.now()

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = frame_id

        imu_msg.linear_acceleration.x = lin_accel.x * lin_acc_rescale
        imu_msg.linear_acceleration.y = lin_accel.y * lin_acc_rescale
        imu_msg.linear_acceleration.z = lin_accel.z * lin_acc_rescale

        # At default sensitivity of 250deg/s we need to scale by 131.
        imu_msg.angular_velocity.x = gyro[0] / 131.
        imu_msg.angular_velocity.y = gyro[1] / 131.
        imu_msg.angular_velocity.z = gyro[2] / 131.

        imu_pub.publish(imu_msg)

        r.sleep()
