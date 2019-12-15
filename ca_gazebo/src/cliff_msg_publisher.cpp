#include "ca_gazebo/cliff_msg_publisher.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboCliffMsgPublisher)

void GazeboCliffMsgPublisher::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->update_period_ = 1.0 / (_sdf->Get<double>("updateRate"));
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "cliff_msg_publisher");
  }
  // Initialize the ros variables and gazebo variables
  this->robot_name_ = _sdf->Get<std::string>("robotNumber");
  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("cliff plugin missing <frameName>, defaults to world");
    this->frame_name_ = "base_footprint";
  }
  else
  {
    this->frame_name_ = _sdf->Get<std::string>("frameName");
  }
  this->msg_.header.frame_id = this->frame_name_;
  this->side_left_sub_ = this->nh_.subscribe(
      this->robot_name_ + "/side_left_cliff_sensor/raw/", 1, &GazeboCliffMsgPublisher::SideLeftCliffCallback, this);
  this->side_right_sub_ = this->nh_.subscribe(
      this->robot_name_ + "/side_right_cliff_sensor/raw/", 1, &GazeboCliffMsgPublisher::SideRightCliffCallback, this);
  this->front_left_sub_ = this->nh_.subscribe(
      this->robot_name_ + "/front_left_cliff_sensor/raw/", 1, &GazeboCliffMsgPublisher::FrontLeftCliffCallback, this);
  this->front_right_sub_ = this->nh_.subscribe(
      this->robot_name_ + "/front_right_cliff_sensor/raw/", 1, &GazeboCliffMsgPublisher::FrontRightCliffCallback, this);
  this->publisher_ = this->nh_.advertise<ca_msgs::Cliff>(this->robot_name_ + "/cliff_msg/", 1);
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboCliffMsgPublisher::OnUpdate, this));
  this->prev_update_time_ = ros::Time::now();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboCliffMsgPublisher::OnUpdate()
//Every update check if one of the sensors is active, if it is, change the output to true, publishes every 1ms
{
  if ((ros::Time::now() - this->prev_update_time_) < ros::Duration(this->update_period_))
  {
    return;
  }
  this->msg_.header.seq++;
  this->msg_.header.stamp = ros::Time::now();
  {
    const std::lock_guard<std::mutex> lock(side_left_mutex);
    this->msg_.is_cliff_left = this->get_side_left_;
  }
  {
    const std::lock_guard<std::mutex> lock(side_right_mutex);
    this->msg_.is_cliff_right = this->get_side_right_;
  }
  {
    const std::lock_guard<std::mutex> lock(front_left_mutex);
    this->msg_.is_cliff_front_left = this->get_front_left_;
  }
  {
    const std::lock_guard<std::mutex> lock(front_right_mutex);
    this->msg_.is_cliff_front_right = this->get_front_right_;
  }
  this->publisher_.publish(this->msg_);
  this->prev_update_time_ = ros::Time::now();
}

// Callback to get the data from the side left cliff sensor
void GazeboCliffMsgPublisher::SideLeftCliffCallback(const std_msgs::Bool::ConstPtr &data)
{
  const std::lock_guard<std::mutex> lock(side_left_mutex);
  this->get_side_left_ = data->data;
}

// Callback to get the data from the side right cliff sensor
void GazeboCliffMsgPublisher::SideRightCliffCallback(const std_msgs::Bool::ConstPtr &data)
{
  const std::lock_guard<std::mutex> lock(side_right_mutex);
  this->get_side_right_ = data->data;
}

// Callback to get the data from the front left cliff sensor
void GazeboCliffMsgPublisher::FrontLeftCliffCallback(const std_msgs::Bool::ConstPtr &data)
{
  const std::lock_guard<std::mutex> lock(front_left_mutex);
  this->get_front_left_ = data->data;
}

// Callback to get the data from the front right sensor
void GazeboCliffMsgPublisher::FrontRightCliffCallback(const std_msgs::Bool::ConstPtr &data)
{
  const std::lock_guard<std::mutex> lock(front_right_mutex);
  this->get_front_right_ = data->data;
}
} // namespace gazebo
