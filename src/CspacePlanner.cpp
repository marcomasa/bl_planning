#include <pluginlib/class_list_macros.h>

#include "cf_global_planner/CspacePlanner.h"

// register this planner as a BaseGlobalPlanner plugin NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(CspacePlanner, nav_core::BaseGlobalPlanner)

CspacePlanner::CspacePlanner(std::string name, costmap_2d::Costmap2D *costmap,
                             std::string frame_id)
    : global_planner::GlobalPlanner::GlobalPlanner(name, costmap, frame_id) {
  initialize(name, costmap, frame_id);
}

void CspacePlanner::initialize(std::string name,
                               costmap_2d::Costmap2DROS *costmap_ros) {
  costmap_ros_ = costmap_ros;
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void CspacePlanner::initialize(std::string name, costmap_2d::Costmap2D *costmap,
                               std::string frame_id) {
  ROS_INFO_STREAM(logger_prefix_ << "Init as '" << name << "'");
  global_planner::GlobalPlanner::initialize(name, costmap, frame_id);

  if (!is_initialized_) {
    ros::NodeHandle nh("/nav/" + name);
    ros::NodeHandle private_nh("~/" + name);

    is_initialized_ = true;

    // convert our costmap to the simple map type
    ros::Time t_conversion = ros::Time::now();
    ROS_INFO_STREAM(logger_prefix_ << "Converting the costmap to SimpleMap");

    SimpleMap<int> map(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

    for (unsigned int x = 0; x < costmap->getSizeInCellsX(); x++) {
      for (unsigned int y = 0; x < costmap->getSizeInCellsY(); y++) {
        int cell_value = static_cast<int>(costmap->getCost(x, y));

        if(cell_value >= costmap_2d::LETHAL_OBSTACLE)
        {
          map.setCell(x, y, 0);
        }
        else
        {
          map.setCell(x, y, INT_MAX);
        }
      }
    }

    ROS_INFO_STREAM(logger_prefix_ << "Conversion took "
                                   << (ros::Time::now() - t_conversion).toSec()
                                   << "s");

    // seems like the code only supports rectangular robots

    double robot_length = 1.0;
    double robot_width = 1.0;

    std::vector<RobotColumn> columns;
    RobotColumn col;
    col.lower = 0;
    col.upper = 1;
    col.x1 = robot_length / 2.0;
    col.x2 = -robot_length / 2.0;
    col.y1 = -robot_width / 2.0;
    col.y2 = robot_width / 2.0;

    columns.push_back(col);

    cspacevoronoi_ = std::make_shared<CSpaceVoronoi>(columns, &map);
  }
}

bool CspacePlanner::makePlan(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal,
                             std::vector<geometry_msgs::PoseStamped> &plan) {

  ROS_INFO_STREAM_THROTTLE(0.01F, logger_prefix_ << "Making Plan!..");

  ros::Time t_plan = ros::Time::now();

  if (!global_planner::GlobalPlanner::makePlan(start, goal, plan)) {
    ROS_ERROR_STREAM_THROTTLE(0.1F, logger_prefix_
                                        << "Could not calculate a valid plan!");
    return false;
  }

  ROS_INFO_STREAM(logger_prefix_ << "Normal planning took "
                                 << (ros::Time::now() - t_plan).toSec()
                                 << " s");

  // ros_viz_.pubPlan(plan, "plan");

  return true;
}
