#include <cmath>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <ca_gazebo/create_virtual_wall_plugin.hh>

using gazebo::VirtualWallSensorPlugin;

static const char create2_model_name_prefix[] = "irobot_create2.";
static const size_t create2_model_name_prefix_length = sizeof(create2_model_name_prefix) - 1;
static const ros::Duration update_rate = ros::Duration(0.01); // 100 Hz

GZ_REGISTER_SENSOR_PLUGIN(VirtualWallSensorPlugin)

void VirtualWallSensorPlugin::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  this->sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
  this->world_ = physics::get_world();
  this->sensor_->SetActive(true);
  ROS_DEBUG_NAMED("virtual_wall_plugin", "Loading");

  if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "virtual_wall_node");
  }
  this->rosnode_ = std::make_unique<ros::NodeHandle>();
  this->updateNewEntity_ = event::Events::ConnectAddEntity(
    std::bind(&VirtualWallSensorPlugin::OnAddEntity, this));
  this->OnAddEntity();
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&VirtualWallSensorPlugin::OnUpdate, this));
  ROS_DEBUG_NAMED("virtual_wall_plugin", "Loaded");
  this->prev_update_time_ = ros::Time::now();
}

void VirtualWallSensorPlugin::OnAddEntity()
{
  const auto & world_models = this->world_->Models();
  // Filter models which name starts with `irobot_create2.` and are continued with a number.
  create_2_models_.resize(0);
  pubs_.resize(0);
  std::copy_if(
    world_models.begin(),
    world_models.end(),
    std::back_inserter(this->create_2_models_),
    [this] (const physics::ModelPtr & model)
    {
      const std::string & name = model->GetName();
      ROS_DEBUG_NAMED("virtual_wall_plugin", "checking model %s", name.c_str());
      if (name.size() < create2_model_name_prefix_length + 1) {
        ROS_DEBUG_NAMED("virtual_wall_plugin", "model name `%s` is short", name.c_str());
        return false;
      }
      if (std::strncmp(
        name.c_str(),
        create2_model_name_prefix,
        create2_model_name_prefix_length) != 0)
      {
        ROS_DEBUG_NAMED(
          "virtual_wall_plugin",
          "model name `%s` doesn't start with prefix `%s`",
          name.c_str(),
          create2_model_name_prefix);
        return false;
      }
      std::istringstream iss(name.substr(create2_model_name_prefix_length));
      size_t x;
      iss >> x;
      if (iss && iss.eof()) {
        std::ostringstream oss;
        oss << "create" << x << "/virtual_wall/raw/";
        ROS_DEBUG_NAMED("virtual_wall_plugin", "model name `%s` ok", name.c_str());
        this->pubs_.emplace_back(
          this->rosnode_->advertise<std_msgs::Bool>(
            oss.str().c_str(), 1));
        return true;
      }
      ROS_DEBUG_NAMED("virtual_wall_plugin", "model name `%s` doesn't end with a number", name.c_str());
      return false;
    });
  ROS_DEBUG_NAMED("virtual_wall_plugin", "number of create2: %zu", create_2_models_.size());
}

void VirtualWallSensorPlugin::OnUpdate()
{
  if ((ros::Time::now() - this->prev_update_time_) < update_rate) {
    return;
  }
  double detected_range = this->sensor_->Range(0);
  ignition::math::Vector3d laser_dir(1., 0., 0.);
  ignition::math::Vector3d sensor_offset(0.145, 0., 0.068);
  const auto & parent_pose = world_->EntityByName(sensor_->ParentName())->WorldPose();
  const ignition::math::Pose3d & laser_pose = parent_pose + sensor_->Pose();

  laser_dir = laser_pose.Rot().RotateVector(laser_dir);
  ROS_DEBUG_NAMED("virtual_wall_plugin", "pose x=%f y=%f z=%f",
    laser_pose.Pos().X(),
    laser_pose.Pos().Y(),
    laser_pose.Pos().Z());
  ROS_DEBUG_NAMED("virtual_wall_plugin", "dir x=%f y=%f z=%f",
    laser_dir.X(),
    laser_dir.Y(),
    laser_dir.Z());

  for (size_t i = 0; i < create_2_models_.size(); i++) {
    const auto & model = this->create_2_models_[i];
    const auto & pub = this->pubs_[i];

    const ignition::math::Pose3d & robot_pose = model->WorldPose();
    sensor_offset = robot_pose.Rot().RotateVector(sensor_offset);
    ignition::math::Pose3d sensor_pose = robot_pose + sensor_pose;
    ROS_DEBUG_NAMED("virtual_wall_plugin", "sensor_pose x=%f y=%f z=%f",
        sensor_pose.Pos().X(),
        sensor_pose.Pos().Y(),
        sensor_pose.Pos().Z());

    // distance from P to straight line X=X0+lambda*V is:
    // sin(phi)*abs(P-X0) where cos(phi)=(P-X0)*V/abs(P-X0)
    const auto & error = (sensor_pose.Pos() - laser_pose.Pos());
    double phi = std::acos(error.Dot(laser_dir) / error.Length());
    double d = error.Length() * std::sin(phi);
    double lambda = (sensor_pose.Pos().X() - laser_pose.Pos().X()) / laser_dir.X();
    ROS_DEBUG_NAMED("virtual_wall_plugin", "d=%f lambda=%f detected_range=%f error=%f",
        d,
        lambda,
        detected_range,
        error.Length());

    std_msgs::Bool msg;
    bool in_range = detected_range > (error.Length() - 0.1) && error.Length() < this->sensor_->RangeMax();
    msg.data = d < 0.1 && lambda > 0 && in_range;
    ROS_DEBUG_NAMED("virtual_wall_plugin", "result=%d",
      msg.data);
    pub.publish(msg);
  }
  this->prev_update_time_ = ros::Time::now();
}
