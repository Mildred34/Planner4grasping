#include "robot_simulation_utilities/object_simulation.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ObjectSimulation");

ObjectSimulation::ObjectSimulation()
{
    RCLCPP_INFO(LOGGER, "Initialization unsuccessful.");
}


ObjectSimulation::ObjectSimulation(const ObjectSimulation& copy)
{
    _name = copy._name;
}


ObjectSimulation::ObjectSimulation(string name)
{
    _name = name;

    RCLCPP_INFO(LOGGER, "Initialization successful.");
}

int ObjectSimulation::ResetSimulation()
{

    RCLCPP_INFO(LOGGER, "Reset was successful.");

    return 1;
}


