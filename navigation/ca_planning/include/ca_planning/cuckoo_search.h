#ifndef __CA_PLANNING__CUCKOO_SEARCH_PLANNER_H__
#define __CA_PLANNING__CUCKOO_SEARCH_PLANNER_H__

#include <ros/ros.h>
// http://wiki.ros.org/costmap_2d
// http://docs.ros.org/melodic/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html
// This map will be accessed automatically by the path planner class when defined as plugin. There is no need to subscribe to costmap2d to get the cost map from ROS.
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
// http://docs.ros.org/melodic/api/nav_core/html/base__global__planner_8h.html
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <boost/make_shared.hpp>

#include "cuckoo_search_lib/FunctionHelper.h"
#include "cuckoo_search_lib/CuckooSearch.h"


namespace cuckoo_search_planner
{
// http://docs.ros.org/melodic/api/nav_core/html/classnav__core_1_1BaseGlobalPlanner.html
class CuckooSearchPlanner
    : public nav_core::BaseGlobalPlanner
{
private:
  bool initialized_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  double step_size_, min_dist_from_robot_;
  base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
public:
  CuckooSearchPlanner();
  // Used to initialize the costmap, that is the map that will be used for planning (costmap_ros), and the name of the planner (name).
  CuckooSearchPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
};  // class CuckooSearchPlanner
};  // namespace cuckoo_search_planner

#endif  // __CA_PLANNING__CUCKOO_SEARCH_PLANNER_H__
