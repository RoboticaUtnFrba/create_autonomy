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
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

    // Define objective function
    std::function<double(std::valarray<double>)> function = CuckooSearchPlanner::distance;
    unsigned int dimensions(2);
    of.ChangeFunction(function);
    of.SetDimensions(dimensions);
    of.SetName("Cuckoo Search Planner");
    // Stop criterian
    StopCritearian stop_criterian = CuckooSearchPlanner::stopCriterian;
    // Other params
    unsigned amount_of_nests = 32;
    Step step = 1.0;
    Lambda lambda = {0.3, 1.99};
    double prob = 0.25;
    unsigned max_generations = 10000;
    bool use_lazy_cuckoo = false;
    // Initialize Cuckoo Search planner
    cs_planner_ = boost::make_shared<CuckooSearch>(of,
        amount_of_nests, step, lambda, prob,
        max_generations, stop_criterian, use_lazy_cuckoo);

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


bool CuckooSearchPlanner::stopCriterian() {
  return true;
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

  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

  Bounds bounds;
  bounds.lower_bound = 0.0;
  bounds.upper_bound = costmap->getSizeInMetersX();
  of.SetBounds(bounds);

  // The final plan will be stored here
  // This plan will be automatically published through the plugin as a topic.
  plan.push_back(start);

  geometry_msgs::PoseStamped new_goal = goal;
  plan.push_back(new_goal);
  plan.push_back(goal);
  return true;
}


};  // namespace cuckoo_search_planner
