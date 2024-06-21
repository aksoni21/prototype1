#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <bot_msgs/action/bot_task_action.hpp>
#include <memory>
#include <thread>

using namespace std::placeholders;

namespace bot_taskserver
{
    class TaskServer : public rclcpp::Node
    {
    public:
        explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("task_server", options)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pre ---Action server ready.");
            action_server_ = rclcpp_action::
                create_server<bot_msgs::action::BotTaskAction>(this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
                                                               std::bind(&TaskServer::cancelCallback, this, _1), std::bind(&TaskServer::acceptedCallback, this, _1));

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Action server ready.");
        }

    private:
        rclcpp_action::Server<bot_msgs::action::BotTaskAction>::SharedPtr action_server_;
        rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const bot_msgs::action::BotTaskAction::Goal> goal)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal request with id: " << goal->task_number);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        void acceptedCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bot_msgs::action::BotTaskAction>> goal_handle)
        {
            std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bot_msgs::action::BotTaskAction>> goal_handle)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal");
            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_group");
            auto handle_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "handle_group");

            std::vector<double> arm_joint_goal;
            std::vector<double> handle_joint_goal;

            if (goal_handle->get_goal()->task_number == 0)
            {
                arm_joint_goal = {0.0, 0.0, 0.0, 0.0};
                handle_joint_goal = {-0.0};
            }
            else if (goal_handle->get_goal()->task_number == 1)
            {
                arm_joint_goal = {-1.14, -0.6, 1.57, -0.7};
                handle_joint_goal = {0.7};
            }
            else if (goal_handle->get_goal()->task_number == 2)
            {
                arm_joint_goal = {-1.57, 1.57, 1.0, -1.0};
                handle_joint_goal = {-0.8};
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid Task Number");
                return;
            }

            bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
            bool handle_within_bounds = handle_move_group.setJointValueTarget(handle_joint_goal);

            if (!arm_within_bounds | !handle_within_bounds)
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target Joint values out of bounds");
                return;
            }

            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Kinematics solver: %s",
                        arm_move_group.getPlanningPipelineId().c_str());
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Planning group: %s",
                        arm_move_group.getName().c_str());
            
            moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
            moveit::planning_interface::MoveGroupInterface::Plan handle_plan;

            bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
            bool handle_plan_success = handle_move_group.plan(handle_plan) == moveit::core::MoveItErrorCode::SUCCESS;

            if (arm_plan_success && handle_plan_success)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning Succeeded and now moving the arm and handle");
                arm_move_group.move();
                handle_move_group.move();
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed");
                return;
            }

            auto result = std::make_shared<bot_msgs::action::BotTaskAction::Result>();
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Succeeded");
        }
        rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bot_msgs::action::BotTaskAction>> goal_handle)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal Cancel");
            auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm_group");
            auto handle_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "handle_group");

            arm_move_group.stop();
            handle_move_group.stop();
            return rclcpp_action::CancelResponse::ACCEPT;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(bot_taskserver::TaskServer)