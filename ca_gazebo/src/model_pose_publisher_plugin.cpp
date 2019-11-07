#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sdf/sdf.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

static const ros::Duration update_period = ros::Duration(0.01); // 10 ms
namespace gazebo
{
class ModelPosePublisherPlugin : public ModelPlugin
{

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      GZ_ASSERT(_parent, "ModelPosePublisherPlugin _parent pointer is NULL");

      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      ROS_INFO("Model pose publisher started!");

      // Store the pointer to the model
      this->model_ = _parent;

      ROS_INFO("model Name = %s", this->model_->GetName().c_str());

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPosePublisherPlugin::OnUpdate, this));
      this->prev_update_time_ = ros::Time::now();

      this->rosnode_.reset(new ros::NodeHandle());
      this->pose_pub_ = this->rosnode_->advertise<geometry_msgs::Pose>((this->model_->GetName() + "/pose").c_str(), 1);
      this->odom_pub_ = this->rosnode_->advertise<nav_msgs::Odometry>((this->model_->GetName() + "/odom").c_str(), 1);
      this->link_ = this->model_->GetLink("link");

      this->odom_msg_.header.frame_id = kMapFrameID_;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      if ((ros::Time::now() - this->prev_update_time_) < update_period) {
       return;
      }

      // Pose message
      geometry_msgs::PosePtr pose_msg(new geometry_msgs::Pose);
      ignition::math::Pose3d pose = this->link_->WorldPose();
      pose_msg->position.x = pose.Pos().X();
      pose_msg->position.y = pose.Pos().Y();
      pose_msg->position.z = pose.Pos().Z();

      pose_msg->orientation.x = pose.Rot().X();
      pose_msg->orientation.y = pose.Rot().Y();
      pose_msg->orientation.z = pose.Rot().Z();
      pose_msg->orientation.w = pose.Rot().W();

      this->pose_pub_.publish(pose_msg);

      // Odom message
      this->odom_msg_.header.seq += 1;
      this->odom_msg_.header.stamp = ros::Time::now();
      this->odom_msg_.child_frame_id = this->model_->GetName();
      this->odom_msg_.pose.pose = *pose_msg;
      // this->odom_msg_.pose.covariance;
      this->odom_msg_.twist.twist.linear.x = this->link_->WorldLinearVel().X();
      this->odom_msg_.twist.twist.linear.y = this->link_->WorldLinearVel().Y();
      this->odom_msg_.twist.twist.linear.z = this->link_->WorldLinearVel().Z();
      this->odom_msg_.twist.twist.angular.x = this->link_->WorldAngularVel().X();
      this->odom_msg_.twist.twist.angular.y = this->link_->WorldAngularVel().Y();
      this->odom_msg_.twist.twist.angular.z = this->link_->WorldAngularVel().Z();
      // this->odom_msg_.twist.covariance;
      this->odom_pub_.publish(this->odom_msg_);

      // Publish Tf
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id= kMapFrameID_;
      transformStamped.child_frame_id = this->model_->GetName();
      transformStamped.transform.translation.x = pose_msg->position.x;
      transformStamped.transform.translation.y = pose_msg->position.y;
      transformStamped.transform.translation.z = 0.0;
      transformStamped.transform.rotation = pose_msg->orientation;

      this->br_.sendTransform(transformStamped);

      // Update time
      this->prev_update_time_ = ros::Time::now();
    }

    private: std::shared_ptr<ros::NodeHandle> rosnode_;
    private: ros::Publisher pose_pub_;
    private: ros::Publisher odom_pub_;
    private: nav_msgs::Odometry odom_msg_;
    private: physics::ModelPtr model_;
    private: physics::LinkPtr link_;
    private: ros::Time prev_update_time_;
    private: tf2_ros::TransformBroadcaster br_;
    private: const std::string kMapFrameID_ = "map";

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
   };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ModelPosePublisherPlugin)
}
