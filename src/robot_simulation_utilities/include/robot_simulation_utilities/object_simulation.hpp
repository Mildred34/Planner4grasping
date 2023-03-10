#pragma once

// Roscpp
#include "rclcpp/rclcpp.hpp"

// Standard library
#include <string>

using namespace std;

class ObjectSimulation
{
    public:
        // Constructor
        ObjectSimulation();
        ObjectSimulation(string name);

        // Copy Constructor
        ObjectSimulation(const ObjectSimulation& copy);

        // Destructor
        ~ObjectSimulation()=default;

    private:
    
        string _name;
};