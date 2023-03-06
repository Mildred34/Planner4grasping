#include "grasp_capture/create_planning_scene.hpp"

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("MoveItCreatePlanningScene");

MoveItCreatePlanningScene::MoveItCreatePlanningScene() : Node("ex_follow_target"),
                                           move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
  // Use upper joint velocity and acceleration limits
  this->move_group_.setMaxAccelerationScalingFactor(1.0);
  this->move_group_.setMaxVelocityScalingFactor(1.0);

  // Add obstacle to the planning scene
  collision_objects.id = "conveyor";
  collision_objects.header.frame_id = this->move_group_.getPlanningFrame();

  // Create identity rotation quaternion
  tf2::Quaternion zero_orientation;
  zero_orientation.setRPY(0, 0, 0);
  const geometry_msgs::msg::Quaternion zero_orientation_msg = tf2::toMsg(zero_orientation);

  // Define the primitive and its dimensions.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.75;
  primitive.dimensions[primitive.BOX_Y] = 3;
  primitive.dimensions[primitive.BOX_Z] = 0.4;

  collision_objects.primitives.push_back(primitive);

  // Define the pose of the table.
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation = zero_orientation_msg;
  box_pose.position.x = 0.7;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.2;

  collision_objects.primitive_poses.push_back(box_pose);
  collision_objects.operation = collision_objects.ADD;

  // Adding obstacles to the planing scene
  // planning_scene_interface_.applyCollisionObject(collision_objects);
  // planning_scene_interface_.applyCollisionObjects(collision_objects);

  // Publish the collision object to the planning scene
  // publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  // timer_ = this->create_wall_timer(500ms, std::bind(&MoveItCreatePlanningScene::timer_callback, this));

  // Subscribe to target pose
  // target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

  
  RCLCPP_WARN(LOGGER, "Initialization successful.");
}

void MoveItCreatePlanningScene::target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // Return if target pose is unchanged
  if (msg->pose == previous_target_pose_)
  {
    return;
  }

  //RCLCPP_INFO(this->get_logger(), "Target pose has changed. Planning and executing...");
  RCLCPP_WARN(LOGGER, "Target pose has changed. Planning and executing...");

  // Plan and execute motion
  this->move_group_.setPoseTarget(msg->pose);
  this->move_group_.move();

  // Update for next callback
  previous_target_pose_ = msg->pose;
}

void MoveItCreatePlanningScene::timer_callback()
{
  RCLCPP_WARN(LOGGER, "Adding collision object to the planning scene...");

  // Create the message to publish the collision object
  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_objects);
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw;

  publisher_->publish(ps);

  // Publish once the collision object
  timer_->cancel();

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto target_follower = std::make_shared<MoveItCreatePlanningScene>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
