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

        // Reset object position
        int ObjectSimulation::ResetSimulation();

    private:
        string _name;
        string _meshpath;
};