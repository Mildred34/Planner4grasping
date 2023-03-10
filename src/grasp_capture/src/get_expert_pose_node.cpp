#include "grasp_capture/get_expert_pose_node.hpp"

GetPose::GetPose() : Node("get_expert_pose_node")
{
    rclcpp::Parameter name_param = this->get_parameter("object");
    _name = name_param.as_string();
    
    ObjectSimulation object(_name);
   //_object = ObjectSimulation(_name);

    // Get Object Position
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto get_expert_pose_node = std::make_shared<GetPose>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(get_expert_pose_node);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}