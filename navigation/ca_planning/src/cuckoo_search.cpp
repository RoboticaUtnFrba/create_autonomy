#include <ca_planning/cuckoo_search.h>

// Register this planner as a BaseGlobalPlanner plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cuckoo_search_planner::CuckooSearchPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace cuckoo_search_planner
{


CuckooSearchPlanner::CuckooSearchPlanner ()
    : costmap_ros_(NULL), initialized_(false) {}


CuckooSearchPlanner::CuckooSearchPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false) {
  initialize(name, costmap_ros);
}


void CuckooSearchPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if(!this->initialized_) {
    costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
    costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_
    // initialize other planner parameters
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("step_size", step_size_, costmap_->getResolution());
    private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
    this->initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
}


bool CuckooSearchPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan ) {
  if(!this->initialized_){
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }
  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

  plan.clear();
  costmap_ = costmap_ros_->getCostmap();

  // The final plan will be stored here
  // This plan will be automatically published through the plugin as a topic.
  plan.push_back(start);

  geometry_msgs::PoseStamped new_goal = goal;
  plan.push_back(new_goal);
  plan.push_back(goal);
  return true;
}


};  // namespace cuckoo_search_planner
