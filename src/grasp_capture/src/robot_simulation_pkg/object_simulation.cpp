#include "robot_simulation_pkg/object_simulation.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ObjectSimulation");

ObjectSimulation::ObjectSimulation(const ObjectSimulation& copy)
{
    _name = copy._name;
}


ObjectSimulation::ObjectSimulation(string name)
{
    _name = name;
    RCLCPP_INFO(LOGGER, "Initialization successful.");
}

