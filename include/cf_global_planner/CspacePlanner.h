#pragma once

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

// bl lib
#include "cspacevoronoi.h"

class CspacePlanner : public global_planner::GlobalPlanner {
public:
  CspacePlanner() = default;
  CspacePlanner(std::string name, costmap_2d::Costmap2D *costmap,
                std::string frame_id);

  // overridden functions from interface global_planner::GlobalPlanner
  void initialize(std::string name,
                  costmap_2d::Costmap2DROS *costmap_ros) override;
  void initialize(std::string name, costmap_2d::Costmap2D *costmap,
                  std::string frame_id);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan) override;

private:
  std::string logger_prefix_ = "[CFP]\t";

  bool is_initialized_ = false;
  costmap_2d::Costmap2DROS *costmap_ros_ = nullptr;

  std::shared_ptr<CSpaceVoronoi> cspacevoronoi_;
};
// namespace evo_nav_plan
