#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <ca_planning/navfn.h>
#include <ca_planning/dmp.h>
#include <ca_planning/jps_planner/jps_planner.h>


namespace jps {
  /**
   * @class JumpPointSearchROS
   * @brief Provides a ROS wrapper for the jps planner which <<<TODO>>>.
   */
  class JumpPointSearchROS : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Default constructor for the JumpPointSearchROS object
       */
      JumpPointSearchROS();

      /**
       * @brief  Constructor for the JumpPointSearchROS object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      JumpPointSearchROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Constructor for the JumpPointSearchROS object
       * @param  name The name of this planner
       * @param  costmap A pointer to the costmap to use
       * @param  global_frame The global frame of the costmap
       */
      JumpPointSearchROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

      /**
       * @brief  Initialization function for the JumpPointSearchROS object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the JumpPointSearchROS object
       * @param  name The name of this planner
       * @param  costmap A pointer to the costmap to use for planning
       * @param  global_frame The global frame of the costmap
       */
      void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param tolerance The tolerance on the goal point for the planner
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
          const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

      ~JumpPointSearchROS(){}

      bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

      /**
       * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
       */
      // <<< TODO: Use smart pointer!!! >>>
      costmap_2d::Costmap2D* costmap_;
      std::shared_ptr<JPS::OccMapUtil> map_util_;
      std::shared_ptr<JPSPlanner2D> jps_planner_;
      std::shared_ptr<DMPlanner2D> dmp_planner_;
      ros::Publisher plan_pub_;
      bool initialized_;

    private:
      void mapToWorld(double mx, double my, double& wx, double& wy);
      void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
      double default_tolerance_;
      bool debug_;
      boost::mutex mutex_;
      ros::ServiceServer make_plan_srv_;
      std::string global_frame_;
      bool solution_found_ = false;
  };
};
