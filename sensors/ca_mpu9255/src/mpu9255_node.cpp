#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include "sensor_msgs/MagneticField.h"
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <fstream>

//gyroscope offset
const uint16_t XG_OFFSET_H = 0x13;
const uint16_t XG_OFFSET_L = 0x14;
const uint16_t YG_OFFSET_H = 0x15;
const uint16_t YG_OFFSET_L = 0x16;
const uint16_t ZG_OFFSET_H = 0x17;
const uint16_t ZG_OFFSET_L = 0x18;

//accelerometer offset
const uint16_t XA_OFFSET_H = 0x77;
const uint16_t XA_OFFSET_L = 0x78;
const uint16_t YA_OFFSET_H = 0x7A;
const uint16_t YA_OFFSET_L = 0x7B;
const uint16_t ZA_OFFSET_H = 0x7D;
const uint16_t ZA_OFFSET_L = 0x7E;

void readFactoryOffsets(int &fd,
  std::vector<int16_t> &factoryGyroOffset, 
  std::vector<int16_t> &factoryAccelOffset) {
  // Read factory gyroscope offset
  factoryGyroOffset.push_back(
      (wiringPiI2CReadReg8(fd, XG_OFFSET_L)<<8) | 
       wiringPiI2CReadReg8(fd, XG_OFFSET_H));
  factoryGyroOffset.push_back(
      (wiringPiI2CReadReg8(fd, YG_OFFSET_L)<<8) | 
       wiringPiI2CReadReg8(fd, YG_OFFSET_H));
  factoryGyroOffset.push_back(
      (wiringPiI2CReadReg8(fd, ZG_OFFSET_L)<<8) | 
       wiringPiI2CReadReg8(fd, ZG_OFFSET_H));

  // Based on http://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers .
  // Read factory accelerometer offset

  // Read the register values and save them as a 16 bit value
  factoryAccelOffset.push_back(
      (wiringPiI2CReadReg8(fd, XA_OFFSET_L)<<8) | 
       wiringPiI2CReadReg8(fd, XA_OFFSET_H));
  factoryAccelOffset.push_back(
      (wiringPiI2CReadReg8(fd, YA_OFFSET_L)<<8) | 
       wiringPiI2CReadReg8(fd, YA_OFFSET_H));
  factoryAccelOffset.push_back(
      (wiringPiI2CReadReg8(fd, ZA_OFFSET_L)<<8) | 
       wiringPiI2CReadReg8(fd, ZA_OFFSET_H));
  // Shift offset values to the right to remove the LSB
  factoryAccelOffset.at(0) >>= 1;
  factoryAccelOffset.at(1) >>= 1;
  factoryAccelOffset.at(2) >>= 1;
}

void readCalibrationFile(std::vector<int16_t> &offset_V) {
  boost::filesystem::path filename(ros::package::getPath("ca_mpu9255"));
  filename /= "config/calibrated.yaml";
  std::cout << "Reading calibration file: " << filename.string() << "\n";
  std::ifstream infile(filename.c_str());
  std::string key;
  float value;
  while(infile >> key >> value) {
    offset_V.push_back(static_cast<int16_t>(value));
  }
}

void setGyroOffsets(int &fd,
  std::vector<int16_t> &factoryGyroOffsets,
  std::vector<int16_t> &offset_V) {
  offset_V.at(0) += factoryGyroOffsets.at(0);
  wiringPiI2CWriteReg8(fd, XG_OFFSET_L, (offset_V.at(0) & 0xFF));
  wiringPiI2CWriteReg8(fd, XG_OFFSET_H, (offset_V.at(0)>>8));
  
  offset_V.at(1) += factoryGyroOffsets.at(1);
  wiringPiI2CWriteReg8(fd, YG_OFFSET_L, (offset_V.at(1) & 0xFF));
  wiringPiI2CWriteReg8(fd, YG_OFFSET_H, (offset_V.at(1)>>8));
  
  offset_V.at(2) += factoryGyroOffsets.at(2);
  wiringPiI2CWriteReg8(fd, ZG_OFFSET_L, (offset_V.at(2) & 0xFF));
  wiringPiI2CWriteReg8(fd, ZG_OFFSET_H, (offset_V.at(2)>>8));
}

void setAccelOffsets(int &fd,
  std::vector<int16_t> &factoryAccelOffsets,
  std::vector<int16_t> &offset_V) {
  offset_V.at(3) += factoryAccelOffsets.at(0);
  wiringPiI2CWriteReg8(fd, XA_OFFSET_L, (offset_V.at(3) & 0xFF)<<1);
  wiringPiI2CWriteReg8(fd, XA_OFFSET_H, (offset_V.at(3)>>7));
  
  offset_V.at(4) += factoryAccelOffsets.at(1);
  wiringPiI2CWriteReg8(fd, YA_OFFSET_L, (offset_V.at(4) & 0xFF)<<1);
  wiringPiI2CWriteReg8(fd, YA_OFFSET_H, (offset_V.at(4)>>7));
  
  offset_V.at(5) += factoryAccelOffsets.at(2);
  wiringPiI2CWriteReg8(fd, ZA_OFFSET_L, (offset_V.at(5) & 0xFF)<<1);
  wiringPiI2CWriteReg8(fd, ZA_OFFSET_H, (offset_V.at(5)>>7));
}

void setOffsets(int &fd) {
  std::vector<int16_t> factoryGyroOffsets;
  std::vector<int16_t> factoryAccelOffsets;
  // Get factory offset
  readFactoryOffsets(fd, factoryGyroOffsets, factoryAccelOffsets);
  // Read offsets from calibration YAML file
  std::vector<int16_t> offset_V;
  readCalibrationFile(offset_V);
  // Set offsets
  setGyroOffsets(fd, factoryGyroOffsets, offset_V);
  setAccelOffsets(fd, factoryAccelOffsets, offset_V);
}

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

  setOffsets(fd);

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
