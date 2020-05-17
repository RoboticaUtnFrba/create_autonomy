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
    costmap_ros_ = costmap_ros;
    // initialize other planner parameters
    ros::NodeHandle private_nh("~/" + name);
    // step_size_ = costmap_->getResolution();
    // world_model_ = new base_local_planner::CostmapModel(*costmap_);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

    // Define objective function
    std::function<double(std::valarray<double>)> function = CuckooSearchPlanner::distance;
    unsigned int dimensions;
    Bounds bounds;
    ObjectiveFunction of(function, dimensions, bounds, "Cuckoo Search Planner");
    cs_planner_ = boost::make_shared<CuckooSearch>(of);

    this->initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
}


double CuckooSearchPlanner::distance(std::valarray<double> va)
{
  const double begin(*std::begin(va));
  const double end(*std::end(va));
  return std::sqrt(std::pow(end - begin, 2));
}


bool CuckooSearchPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan ) {
  // Don't plan simultaneously
  boost::mutex::scoped_lock lock(mutex_);

  if(!this->initialized_){
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }
  // Clear the plan, just in case
  plan.clear();

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

  // costmap_ = costmap_ros_->getCostmap();

  // The final plan will be stored here
  // This plan will be automatically published through the plugin as a topic.
  plan.push_back(start);

  geometry_msgs::PoseStamped new_goal = goal;
  plan.push_back(new_goal);
  plan.push_back(goal);
  return true;
}


};  // namespace cuckoo_search_planner
