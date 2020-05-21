#include <ca_planning/jps_ros.h>
// #include <jps/MakeNavPlan.h>

#include <boost/shared_ptr.hpp>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>


namespace jps {

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

void JumpPointSearchNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
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

int main (int argc, char** argv)
{
  ros::init(argc, argv, "jps_global_planner");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS lcr("costmap", buffer);

  jps::JumpPointSearchNode jps("jps_planner", &lcr);

  ros::spin();
  return 0;
}
