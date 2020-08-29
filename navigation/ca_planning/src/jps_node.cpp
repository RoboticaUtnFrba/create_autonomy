/*
 * Copyright (c) 2020, Emiliano Javier Borghi Orue.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Emiliano Javier Borghi Orue nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ca_planning/jps_ros.h>
// #include <jps/MakeNavPlan.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>


namespace jps
{
class JumpPointSearchNode : public JumpPointSearchROS
{
public:
  JumpPointSearchNode(std::string name, costmap_2d::Costmap2DROS* cmap);
  // bool makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp);

private:
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
  costmap_2d::Costmap2DROS* cmap_;
  // ros::ServiceServer make_plan_service_;
  ros::Subscriber pose_sub_;
};


// bool JumpPointSearchNode::makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp)
// {
//   std::vector<geometry_msgs::PoseStamped> path;

//   req.start.header.frame_id = "map";
//   req.goal.header.frame_id = "map";
//   bool success = makePlan(req.start, req.goal, path);
//   resp.plan_found = success;
//   if (success) {
//     resp.path = path;
//   }

//   return true;
// }

void JumpPointSearchNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  geometry_msgs::PoseStamped global_pose;
  cmap_->getRobotPose(global_pose);
  std::vector<geometry_msgs::PoseStamped> path;
  makePlan(global_pose, *goal, path);
}


JumpPointSearchNode::JumpPointSearchNode(std::string name, costmap_2d::Costmap2DROS* cmap)
    : JumpPointSearchROS(name, cmap)
{
  ros::NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~");
  cmap_ = cmap;
  // make_plan_service_ = pnh->advertiseService("make_plan", &JumpPointSearchNode::makePlanService, this);
  pose_sub_ = pnh->subscribe<geometry_msgs::PoseStamped>("goal", 1, &JumpPointSearchNode::poseCallback, this);
}

}  // namespace jps

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jps_global_planner");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS lcr("costmap", buffer);

  jps::JumpPointSearchNode jps("jps_planner", &lcr);

  ros::spin();
  return 0;
}
