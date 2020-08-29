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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <jps_planner/distance_map_planner/distance_map_planner.h>
#include <jps_planner/jps_planner/jps_planner.h>


namespace jps
{
/**
 * @class JumpPointSearchROS
 * @brief Provides a ROS wrapper for the jps planner which <<<TODO>>>.
 */
class JumpPointSearchROS : public nav_core::BaseGlobalPlanner
{
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
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);  // NOLINT(runtime/references)

    /**
     * @brief  Publish a path for visualization purposes
     */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

    ~JumpPointSearchROS();

    bool makePlanService(nav_msgs::GetPlan::Request& req,  // NOLINT(runtime/references)
        nav_msgs::GetPlan::Response& resp);  // NOLINT(runtime/references)

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);

  protected:
    /**
     * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
     */
    // <<< TODO: Use smart pointer!!! >>>
    costmap_2d::Costmap2D* costmap_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    std::shared_ptr<JPS::OccMapUtil> map_util_;
    std::shared_ptr<JPSPlanner2D> jps_planner_;
    std::shared_ptr<DMPlanner2D> dmp_planner_;

    ros::Publisher plan_pub_;
    ros::Publisher jps_path_pub_;
    ros::Subscriber pose_sub_;
    bool initialized_;

  private:
    void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
    bool debug_;
    boost::mutex mutex_;
    ros::ServiceServer make_plan_srv_;
    std::string global_frame_;
};
};  // namespace jps
