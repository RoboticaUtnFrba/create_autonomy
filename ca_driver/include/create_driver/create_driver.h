/**
Software License Agreement (BSD)
\file      create_driver.h
\authors   Jacob Perron <jacobmperron@gmail.com>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CREATE_AUTONOMY_CREATE_DRIVER_H
#define CREATE_AUTONOMY_CREATE_DRIVER_H

#include <memory>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "create/create.h"
#include "ca_msgs/ChargingState.h"
#include "ca_msgs/Mode.h"
#include "ca_msgs/Bumper.h"
#include "ca_msgs/Wheeldrop.h"
#include "ca_msgs/Cliff.h"
#include "ca_msgs/Overcurrent.h"

namespace create
{

static const uint8_t SONG_0_LENGTH = 2;
static const uint8_t SONG_0_NOTES [] = {64,54};
static const float SONG_0_DURATIONS [] = {1.0,1.0};

static const uint8_t SONG_1_LENGTH = 16;
static const uint8_t SONG_1_NOTES [] = {105,103,100,96,98,107,101,108,105,103,100,96,98,107,103,108};
static const float SONG_1_DURATIONS [] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};

static const uint8_t SONG_2_LENGTH = 16;
static const uint8_t SONG_2_NOTES [] = {84,107,84,107,84,107,84,107,84,107,84,107,84,107,84,107};
static const float SONG_2_DURATIONS [] = {0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};

static const uint8_t SONG_3_LENGTH = 11;
static const uint8_t SONG_3_NOTES [] = {59,59,59,59,62,61,61,59,59,59,59};
static const float SONG_3_DURATIONS [] = {1.0,0.75,0.25,1.0,0.75,0.25,0.7,0.25,0.7,0.25,1.0};

static const uint8_t SONG_4_LENGTH = 16;
static const uint8_t SONG_4_NOTES [] = {79,86,84,83,81,91,86,84,83,81,91,86,84,83,84,81};
static const float SONG_4_DURATIONS [] = {0.9,0.8,0.2,0.2,0.2,0.8,0.7,0.2,0.2,0.2,0.8,0.7,0.2,0.2,0.2,0.9};

// Unused covariances must have a large value
static const double MAX_DBL = 1e10;
static const double COVARIANCE[36] = {1e-5, 0.0,  0.0,     0.0,     0.0,     0.0,
                                      1e-5, 0.0,  0.0,     0.0,     0.0,     0.0,
                                      0.0,  0.0,  MAX_DBL, 0.0,     0.0,     0.0,
                                      0.0,  0.0,  0.0,     MAX_DBL, 0.0,     0.0,
                                      0.0,  0.0,  0.0,     0.0,     MAX_DBL, 0.0,
                                      0.0,  0.0,  0.0,     0.0,     0.0,     1e-3};

class CreateDriver
{
private:
  std::unique_ptr<create::Create> robot_;
  create::RobotModel model_;
  tf::TransformBroadcaster tf_broadcaster_;
  diagnostic_updater::Updater diagnostics_;
  ca_msgs::Mode mode_msg_;
  ca_msgs::ChargingState charging_state_msg_;
  ca_msgs::Bumper bumper_msg_;
  ca_msgs::Cliff cliff_msg_;
  ca_msgs::Wheeldrop wheeldrop_msg_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::PoseWithCovarianceStamped angle_msg_;
  geometry_msgs::TransformStamped tf_odom_;
  ros::Time last_cmd_vel_time_;
  std_msgs::Empty empty_msg_;
  std_msgs::Float32 float32_msg_;
  std_msgs::UInt16 uint16_msg_;
  std_msgs::Int16 int16_msg_;
  sensor_msgs::JointState joint_state_msg_;
  std_msgs::Bool is_wall_msg_;
  ca_msgs::Overcurrent is_overcurrent_msg_;
  bool is_running_slowly_;
  double orientation_;

  // ROS params
  double loop_hz_;
  std::string dev_;
  int baud_;
  double latch_duration_;
  bool publish_tf_;

  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
  void debrisLEDCallback(const std_msgs::BoolConstPtr& msg);
  void spotLEDCallback(const std_msgs::BoolConstPtr& msg);
  void dockLEDCallback(const std_msgs::BoolConstPtr& msg);
  void checkLEDCallback(const std_msgs::BoolConstPtr& msg);
  void powerLEDCallback(const std_msgs::UInt8MultiArrayConstPtr& msg);
  void setASCIICallback(const std_msgs::UInt8MultiArrayConstPtr& msg);
  void playSongCallback(const std_msgs::UInt8ConstPtr& msg);
  void dockCallback(const std_msgs::EmptyConstPtr& msg);
  void undockCallback(const std_msgs::EmptyConstPtr& msg);
  void mainMotorCallback(const std_msgs::Float32ConstPtr& msg);

  bool update();
  void updateBatteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateSafetyDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void publishOdom();
  void publishAngle();
  void publishJointState();
  void publishBatteryInfo();
  void publishButtonPresses() const;
  void publishOmniChar();
  void publishMode();
  void publishBumperInfo();
  void publishCliffInfo();
  void publishWheeldrop();
  void publishIsWall();
  void publishOvercurrent();

  inline float normalizeAngle(float angle) {
    angle = std::fmod(angle + M_PI,2*M_PI);
    if (angle < 0)
        angle += 2*M_PI;
    return angle - M_PI;
  };

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber debris_led_sub_;
  ros::Subscriber spot_led_sub_;
  ros::Subscriber dock_led_sub_;
  ros::Subscriber check_led_sub_;
  ros::Subscriber power_led_sub_;
  ros::Subscriber set_ascii_sub_;
  ros::Subscriber play_song_sub_;
  ros::Subscriber dock_sub_;
  ros::Subscriber undock_sub_;
  ros::Subscriber main_motor_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher angle_pub_;
  ros::Publisher clean_btn_pub_;
  ros::Publisher day_btn_pub_;
  ros::Publisher hour_btn_pub_;
  ros::Publisher min_btn_pub_;
  ros::Publisher dock_btn_pub_;
  ros::Publisher spot_btn_pub_;
  ros::Publisher voltage_pub_;
  ros::Publisher current_pub_;
  ros::Publisher charge_pub_;
  ros::Publisher charge_ratio_pub_;
  ros::Publisher capacity_pub_;
  ros::Publisher temperature_pub_;
  ros::Publisher charging_state_pub_;
  ros::Publisher omni_char_pub_;
  ros::Publisher mode_pub_;
  ros::Publisher bumper_pub_;
  ros::Publisher cliff_pub_;
  ros::Publisher wheeldrop_pub_;
  ros::Publisher wheel_joint_pub_;
  ros::Publisher wall_pub_;
  ros::Publisher overcurrent_pub_;

public:
  CreateDriver(ros::NodeHandle& nh, ros::NodeHandle& ph);
  ~CreateDriver();

  virtual void spin();
  virtual void spinOnce();

};  // class CreateDriver

} //namespace ccreate

#endif  // CREATE_AUTONOMY_CREATE_DRIVER_H
