#pragma once

// Moveit2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <string>

// Roscpp
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/qos.hpp>


/// Example that uses MoveIt 2 to follow a target inside Ignition Gazebo

const std::string MOVE_GROUP = "arm";

class MoveItFollowTarget : public rclcpp::Node
{
public:
  /// Constructor
  MoveItFollowTarget();

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  /// Subscriber for target pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  /// Target pose that is used to detect changes
  geometry_msgs::msg::Pose previous_target_pose_;

private:
  /// Callback that plans and executes trajectory each time the target pose is changed
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
};