#ifndef CA_GAZEBO_CREATE_BATTERY_PLUGIN_H
#define CA_GAZEBO_CREATE_BATTERY_PLUGIN_H

/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2021
 *     Emiliano Borghi
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
/*
 * Desc: Battery plugin
 * Author: Emiliano Borghi
 * Date: 22 Jan 2021
 */

#include <algorithm>
#include <assert.h>
#include <boost/bind.hpp>
#include <map>
#include <mutex>
#include <string>
#include <vector>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <sdf/sdf.hh>
#include <sensor_msgs/BatteryState.h>

#include <ca_gazebo/SetTemperature.h>
#include <ca_gazebo/SetCharge.h>
#include <ca_gazebo/Reset.h>

namespace gazebo
{
class GazeboRosBattery : public ModelPlugin
{
public:
  GazeboRosBattery();
  ~GazeboRosBattery();
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  void Reset();

protected:
  virtual void updateChild();
  virtual void finiChild();
  ros::ServiceServer set_temperature;
  ros::ServiceServer set_charge_state;
  ros::ServiceServer reset_model;
  std::mutex service_lock;

private:
  GazeboRosPtr gazebo_ros_;
  event::ConnectionPtr update_connection_;
  physics::ModelPtr parent_;
  ros::Publisher battery_state_publisher_;
  ros::Publisher battery_voltage_publisher_;
  std::vector<ros::Subscriber> current_draw_subscribers_;
  sensor_msgs::BatteryState battery_state_;
  common::Time last_update_time_;
  std::string battery_topic_;
  std::string consumer_topic_;
  std::string battery_voltage_topic_;
  std::string frame_id_;
  std::string plugin_name_;
  std::vector<double> drawn_currents_;
  // common parameters
  bool publish_voltage_;
  int technology_;
  int num_of_consumers_;
  int cell_count_;
  double update_rate_;
  double update_period_;
  double design_capacity_;
  double current_drawn_;
  double nominal_voltage_;
  double constant_voltage_;
  double cut_off_voltage_;
  double internal_resistance_;
  double lpf_tau_;
  // linear model params
  double lin_discharge_coeff_;
  bool use_nonlinear_model_;
  // nonlinear model params
  double polarization_constant_;        // polarization constant [V/Ah] or pol. resistance [Ohm]
  double exponential_voltage_;          // exponential voltage [V]
  double exponential_capacity_;         // exponential capacity [1/(Ah)]
  double characteristic_time_;          // characteristic time [s] for charge-dependence
  double design_temperature_;           // Design temperature where pol. const is unchanged and voltages are nominal.
  double arrhenius_rate_polarization_;  // Arrhenius rate of polarization constant [K]
  double capacity_temp_coeff_;          // Temperature dependence of capacity [Ah/K]
  double reversible_voltage_temp_;      // Linear temperature dependant voltage shift [V/K]
  // internal variables
  bool model_initialised_;
  bool internal_cutt_off_;
  bool battery_empty_;
  double voltage_;
  double charge_;
  double charge_memory_;
  double current_;
  double discharge_;
  double capacity_;
  double temperature_;
  double temp_set_;
  double temp_lpf_tau_;
  double current_lpf_;

  // model update
  void linearDischargeVoltageUpdate();
  void nonlinearDischargeVoltageUpdate();

  // Services
  bool setCharge(ca_gazebo::SetCharge::Request& req,  // NOLINT(runtime/references)
                 ca_gazebo::SetCharge::Response& res);  // NOLINT(runtime/references)
  bool resetModel(ca_gazebo::Reset::Request& req,  // NOLINT(runtime/references)
                  ca_gazebo::Reset::Response& res);  // NOLINT(runtime/references)
  bool setTemperature(ca_gazebo::SetTemperature::Request& req,  // NOLINT(runtime/references)
                      ca_gazebo::SetTemperature::Response& res);  // NOLINT(runtime/references)

  // Callback Queue
  ros::CallbackQueue queue_;
  std::thread callback_queue_thread_;
  std::mutex lock_;
  void queueThread();
  void currentCallback(const std_msgs::Float32::ConstPtr& current, int consumer_id);
};
}  // namespace gazebo

#endif  // CA_GAZEBO_CREATE_BATTERY_PLUGIN_H
