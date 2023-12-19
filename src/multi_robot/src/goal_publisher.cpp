#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "goals.hpp"

class NavigateToPoseClient : public rclcpp::Node
{
public:
    NavigateToPoseClient()
        : Node("navigate_to_pose_client")
    {
        // Create an ActionClient for the NavigateToPose action
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

        // Wait for the action server to be available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available");
            return;
        }

        // Create a GoalGenerator instance with map width and height as 1
        GoalGenerator goal_generator(1, 1);

        // Generate a random goal position
        GoalPosition goal_position = goal_generator.generateRandomGoal();

        // Create a goal for the NavigateToPose action with constant z and orientation values
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = goal_position.x; //0.21;
        goal_msg.pose.pose.position.y = goal_position.y; //-1.84;
        goal_msg.pose.pose.position.z = 0.0;  // Constant z value
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0000000;

        // Send the goal to the action server
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&NavigateToPoseClient::result_callback, this, std::placeholders::_1);
        auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);

        // Wait for the result
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle);
    }

private:
    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
        }
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToPoseClient>());
    rclcpp::shutdown();
    return 0;
}
