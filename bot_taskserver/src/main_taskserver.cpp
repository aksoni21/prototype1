// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// #include <memory>
// #include <thread>

// using namespace std::placeholders;

// namespace bot_taskserver
// {
//     class TaskServer : public rclcpp::Node
//     {
//     public:
//         explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("task_server", options)
//         {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pre ---Action server ready.");
//             action_server_ = rclcpp_action::
//                 create_server<arduino_bot_msgs::action::ArduinobotTask>(this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
//                                                                         std::bind(&TaskServer::cancelCallback, this, _1), std::bind(&TaskServer::acceptedCallback, this, _1));

//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Action server ready.");
//         }

//     private:
//         rclcpp_action::Server<arduino_bot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
//         rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const arduino_bot_msgs::action::ArduinobotTask::Goal> goal)
//         {
//             RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with id: " << goal->task_number);
//             return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//         }

//         void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduino_bot_msgs::action::ArduinobotTask>> goal_handle)
//         {
//             std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
//         }

//         void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduino_bot_msgs::action::ArduinobotTask>> goal_handle)
//         {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal");
//             auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_group");
//             auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper_group");

//             std::vector<double> arm_joint_goal;
//             std::vector<double> gripper_joint_goal;

//             if (goal_handle->get_goal()->task_number == 0)
//             {
//                 arm_joint_goal = {0.0, 0.0, 0.0};
//                 gripper_joint_goal = {-0.7, -0.7};
                
//             }
//             else if (goal_handle->get_goal()->task_number == 1)
//             {
//                 arm_joint_goal = {-1.14, -0.6, -0.7};
//                 gripper_joint_goal = {0.0, 0.0};
//             }
//             else if (goal_handle->get_goal()->task_number == 2)
//             {
//                 arm_joint_goal = {-1.57, 0.0, -1.0};
//                 gripper_joint_goal = {0.0, 0.0};
//             }
//             else
//             {
//                 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid Task Number");
//                 return;
//             }

//             bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
//             bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

//             if (!arm_within_bounds | !gripper_within_bounds)
//             {
//                 RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target Joint values out of bounds");
//                 return;
//             }

//             moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
//             moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

//             bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
//             bool gripper_plan_success = gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS;

//             if (arm_plan_success && gripper_plan_success)
//             {
//                 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning Succeeded and now moving the arm and gripper");
//                 arm_move_group.move();
//                 gripper_move_group.move();
//             }
//             else
//             {
//                 RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed");
//                 return;
//             }

//             auto result = std::make_shared<arduino_bot_msgs::action::ArduinobotTask::Result>();
//             result->success = true;
//             goal_handle->succeed(result);
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Succeeded");
//         }
//         rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduino_bot_msgs::action::ArduinobotTask>> goal_handle)
//         {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Cancel");
//             auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_group");
//             auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper_group");

//             arm_move_group.stop();
//             gripper_move_group.stop();
//             return rclcpp_action::CancelResponse::ACCEPT;
//         }
//     };
// }

// RCLCPP_COMPONENTS_REGISTER_NODE(arduino_bot_remote::TaskServer)
