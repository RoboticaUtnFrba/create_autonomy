#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include "sensor_msgs/MagneticField.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "IMU_pub");

  ros::NodeHandle n;
  ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("imu/data_raw", 2);
  ros::Publisher pub_mag = n.advertise<sensor_msgs::MagneticField>("imu/mag", 2);

	int fd;
  wiringPiSetupSys();
  fd = wiringPiI2CSetup(0x68);

	if (fd == -1) {
    printf("no i2c device found\n");
    return -1;
	}
  int16_t InBuffer[9] = {0};
  static int32_t OutBuffer[3] = {0};

  while (ros::ok()){
    //http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
    //http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html
    sensor_msgs::Imu data_imu;
    sensor_msgs::MagneticField data_mag;

    data_mag.header.stamp = ros::Time::now();
    data_imu.header.stamp = data_mag.header.stamp;
    data_imu.header.frame_id = "/imu_link";

    float conversion_gyro = 3.1415/(180.0*32.8f);
    float conversion_acce = 9.8/16384.0f;

    //datos acelerómetro
    InBuffer[0]=  (wiringPiI2CReadReg8 (fd, 0x3B)<<8)|wiringPiI2CReadReg8 (fd, 0x3C);
    InBuffer[1]=  (wiringPiI2CReadReg8 (fd, 0x3D)<<8)|wiringPiI2CReadReg8 (fd, 0x3E);
    InBuffer[2]=  (wiringPiI2CReadReg8 (fd, 0x3F)<<8)|wiringPiI2CReadReg8 (fd, 0x40);

    data_imu.linear_acceleration.x = InBuffer[0]*conversion_acce;
    data_imu.linear_acceleration.y = InBuffer[1]*conversion_acce;
    data_imu.linear_acceleration.z = InBuffer[2]*conversion_acce;

     //datos giroscopio
    InBuffer[3]=  (wiringPiI2CReadReg8 (fd, 0x43)<<8)|wiringPiI2CReadReg8 (fd, 0x44);
    InBuffer[4]=  (wiringPiI2CReadReg8 (fd, 0x45)<<8)|wiringPiI2CReadReg8 (fd, 0x46);
    InBuffer[5]=  (wiringPiI2CReadReg8 (fd, 0x47)<<8)|wiringPiI2CReadReg8 (fd, 0x48);

    data_imu.angular_velocity.x = InBuffer[3]*conversion_gyro;
    data_imu.angular_velocity.y = InBuffer[4]*conversion_gyro;
    data_imu.angular_velocity.z = InBuffer[5]*conversion_gyro;

    //datos magnetómetro
    InBuffer[6]=  (wiringPiI2CReadReg8 (fd, 0x04)<<8)|wiringPiI2CReadReg8 (fd, 0x03);
    InBuffer[7]=  (wiringPiI2CReadReg8 (fd, 0x06)<<8)|wiringPiI2CReadReg8 (fd, 0x05);
    InBuffer[8]=  (wiringPiI2CReadReg8 (fd, 0x08)<<8)|wiringPiI2CReadReg8 (fd, 0x07);

    data_mag.magnetic_field.x = InBuffer[6];
    data_mag.magnetic_field.y = InBuffer[7];
    data_mag.magnetic_field.z = InBuffer[8];

    pub_imu.publish(data_imu);

    pub_mag.publish(data_mag);

    ros::spinOnce();
    }
  return 0;
 }
