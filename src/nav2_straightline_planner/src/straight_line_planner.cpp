#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <iostream>

#include "nav2_util/node_utils.hpp"
#include "nav2_straightline_planner/straight_line_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_straightline_planner
{

int threshold = 200;

// Class-level variables
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr binary_map_pub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planner_path_sub_;
nav_msgs::msg::Path latest_external_plan_;

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("StraightLine"), "Node handle is null in configure()");
    return;
  }

  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  // Publishers
  binary_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("binary_map", 1);
  start_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("start_pose", 1);
  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);

  // Subscriber to external path
  planner_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/planner_path", 10,
    [](const nav_msgs::msg::Path::SharedPtr msg) {
      latest_external_plan_ = *msg;
      RCLCPP_INFO(rclcpp::get_logger("StraightLine"),
        "Received planner_path with %lu poses", msg->poses.size());
    });
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s", name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s", name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s", name_.c_str());
}

nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("StraightLine"), "Node handle is null in createPlan()");
    return global_path;
  }

  unsigned int width = costmap_->getSizeInCellsX();
  unsigned int height = costmap_->getSizeInCellsY();
  float map_resolution = costmap_->getResolution();

  // Create binary occupancy grid from costmap
  std::vector<std::vector<int>> binary_map(height, std::vector<int>(width, 0));
  for (unsigned int row = 0; row < height; row++) {
    for (unsigned int col = 0; col < width; col++) {
      unsigned char cost = costmap_->getCost(col, row);
      binary_map[row][col] = (cost >= threshold) ? 1 : 0;
    }
  }

  // Publish binary map as OccupancyGrid
  nav_msgs::msg::OccupancyGrid occ_grid;
  occ_grid.header.stamp = node_->now();
  occ_grid.header.frame_id = global_frame_;
  occ_grid.info.resolution = map_resolution;
  occ_grid.info.width = width;
  occ_grid.info.height = height;
  occ_grid.info.origin.position.x = costmap_->getOriginX();
  occ_grid.info.origin.position.y = costmap_->getOriginY();
  occ_grid.info.origin.position.z = 0.0;
  occ_grid.info.origin.orientation.w = 1.0;
  occ_grid.data.resize(width * height);

  for (unsigned int row = 0; row < height; row++) {
    for (unsigned int col = 0; col < width; col++) {
      int idx = row * width + col;
      occ_grid.data[idx] = binary_map[row][col];
    }
  }
  binary_map_pub_->publish(occ_grid);

  // Convert and publish start and goal
  unsigned int start_col, start_row, goal_col, goal_row;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_col, start_row)) {
    RCLCPP_ERROR(node_->get_logger(), "Start pose is out of costmap bounds!");
    return global_path;
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_col, goal_row)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal pose is out of costmap bounds!");
    return global_path;
  }

 double start_wx, start_wy, goal_wx, goal_wy;
  costmap_->mapToWorld(start_col, start_row, start_wx, start_wy);
  costmap_->mapToWorld(goal_col, goal_row, goal_wx, goal_wy);

  geometry_msgs::msg::PoseStamped start_pub_pose, goal_pub_pose;

  // Start
  start_pub_pose.header.stamp = node_->now();
  start_pub_pose.header.frame_id = global_frame_;
  start_pub_pose.pose.position.x = start_wx;
  start_pub_pose.pose.position.y = start_wy;
  start_pub_pose.pose.position.z = 0.0;
  start_pub_pose.pose.orientation.w = 1.0;
  start_pub_->publish(start_pub_pose);

  // Goal
  goal_pub_pose.header.stamp = node_->now();
  goal_pub_pose.header.frame_id = global_frame_;
  goal_pub_pose.pose.position.x = goal_wx;
  goal_pub_pose.pose.position.y = goal_wy;
  goal_pub_pose.pose.position.z = 0.0;
  goal_pub_pose.pose.orientation.w = 1.0;
  goal_pub_->publish(goal_pub_pose);

  // Use external planner path
  global_path = latest_external_plan_;
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  RCLCPP_INFO(node_->get_logger(), "Returning path with %lu poses from external planner.",
    global_path.poses.size());

  return global_path;
}

}  // namespace nav2_straightline_planner

PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
