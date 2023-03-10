#pragma once

// Roscpp
#include "rclcpp/rclcpp.hpp"
#include "robot_simulation_utilities/object_simulation.hpp"

const std::string MOVE_GROUP = "arm";

class GetPose : public rclcpp::Node
{
public:
  /// Constructor
  GetPose();

private:
    std::string _name;
};