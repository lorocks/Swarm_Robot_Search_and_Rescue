#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "goals.hpp"

class NavigateToPoseNode : public rclcpp::Node
{
public:
    NavigateToPoseNode()
        : Node("navigate_to_pose_client"), goal_index(0)
    {
        // Create an ActionClient for the NavigateToPose action
        goal_send = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

        // Wait for the action server to be available
        if (!goal_send->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return;
        }

        // Define a list of GoalPositions (x, y)
        goal_positions = {
            {0.5, 1.8},
            {0.21, -1.8},
            {0.5, 1.8},
            {0.21, -1.8},
            {0.5, 1.8}
        };

        // Send the first goal
        sendNextGoal();
    }

private:
    void sendNextGoal()
    {
        if (goal_index < goal_positions.size())
        {
            // Create a goal for the NavigateToPose action with constant z and orientation values
            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.pose.position.x = goal_positions[goal_index].x;
            goal_msg.pose.pose.position.y = goal_positions[goal_index].y;
            goal_msg.pose.pose.position.z = 0.0;  // Constant z value
            goal_msg.pose.pose.orientation.x = 0.0;
            goal_msg.pose.pose.orientation.y = 0.0;
            goal_msg.pose.pose.orientation.z = 0.0;
            goal_msg.pose.pose.orientation.w = 1.0000000;

            // Send the goal to the action server
            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&NavigateToPoseNode::result_callback, this, std::placeholders::_1);
            auto future_goal_handle = goal_send->async_send_goal(goal_msg, send_goal_options);

            RCLCPP_INFO(get_logger(), "Sending goal %zu", goal_index);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "All goals achieved!");
        }
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal %zu succeeded!", goal_index);

            // Move to the next goal
            goal_index++;
            sendNextGoal();
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal %zu was aborted", goal_index);

            // Retry the same goal on failure
            sendNextGoal();
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");

            // Move to the next goal
            goal_index++;
            sendNextGoal();
            break;
        }
    }

    std::vector<GoalPosition> goal_positions;
    size_t goal_index;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr goal_send;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create an instance of NavigateToPoseNode
    auto node = std::make_shared<NavigateToPoseNode>();

    auto spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    spinner->add_node(node);

    // Run the spinner in a separate thread
    std::thread spinner_thread([spinner]() {
        spinner->spin();
    });

    // Wait for the spinner to finish
    spinner_thread.join();

    rclcpp::shutdown();

    return 0;
}

