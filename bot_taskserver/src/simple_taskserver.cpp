#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/msg/display_robot_state.h>
// #include <moveit_msgs/msg/display_trajectory.h>
// #include <moveit_msgs/msg/AttachedCollisionObject.h>
// #include <moveit_msgs/msg/CollisionObject.h>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    // auto base_move_group = moveit::planning_interface::MoveGroupInterface(node, "base_group");
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm_group");
    auto handle_move_group = moveit::planning_interface::MoveGroupInterface(node, "handle_group");
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "in move robot 1");
    std::vector<double> arm_joint_goal{-1.57, -0.50, 0.50, 0.80};
    // std::vector<double> base_joint_goal{0.0, 0.0, 1.57};
    std::vector<double> handle_joint_goal{-0.50};

    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "in move robot 2");

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    // bool base_within_bounds = base_move_group.setJointValueTarget(base_joint_goal);
    bool handle_within_bounds = handle_move_group.setJointValueTarget(handle_joint_goal);
    
    // if (!arm_within_bounds | !base_within_bounds | !handle_within_bounds){
        if (!arm_within_bounds ){
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target Goal is not within bounds ");
        return;
    }
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "in move robot 4");

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    // moveit::planning_interface::MoveGroupInterface::Plan base_plan;
    moveit::planning_interface::MoveGroupInterface::Plan handle_plan;

    bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    bool handle_plan_success = handle_move_group.plan(handle_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    // bool base_plan_success = base_move_group.plan(base_plan) == moveit::core::MoveItErrorCode::SUCCESS;

    // if (arm_plan_success && base_plan_success && handle_plan_success){
        if(arm_plan_success ){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner succeeded, moving groups now");
        arm_move_group.move();
        // base_move_group.move();
        handle_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");

    move_robot(node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}