#pragma once

// Moveit2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <string>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Roscpp
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/qos.hpp>

// Other
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

/// Example that uses MoveIt 2 to follow a target inside Ignition Gazebo

const std::string MOVE_GROUP = "arm";

class MoveItCreatePlanningScene : public rclcpp::Node
{
public:
  /// Constructor
  MoveItCreatePlanningScene();

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  /// Subscriber for target pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  /// Target pose that is used to detect changes
  geometry_msgs::msg::Pose previous_target_pose_;

  // 
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit_msgs::msg::CollisionObject collision_objects;

private:
  /// Callback that plans and executes trajectory each time the target pose is changed
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr publisher_;

};