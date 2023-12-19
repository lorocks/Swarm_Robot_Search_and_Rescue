#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "goals.hpp"

class NavigateToPoseNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigateToPoseNode(const std::string &namespace_, GoalGenerator &goalGenerator)
        : Node("navigate_to_pose_node_" + namespace_),
          client_(rclcpp_action::create_client<NavigateToPose>(this, "/" + namespace_ + "/navigate_to_pose")),
          pose_publisher_(create_publisher<geometry_msgs::msg::PoseStamped>("/" + namespace_ + "/goal_pose", 10)),
          timer_(create_wall_timer(std::chrono::seconds(2), std::bind(&NavigateToPoseNode::sendGoal, this))),
          goal_generator_(goalGenerator)  // Pass the GoalGenerator as a reference
    {}

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    GoalGenerator &goal_generator_;  // Reference to the GoalGenerator instance

    void sendGoal()
    {
        auto goal_msg = NavigateToPose::Goal();

        // Use the GoalGenerator to get random x and y positions
        GoalPosition randomGoal = goal_generator_.generateRandomGoal();

        goal_msg.pose.pose.position.x = randomGoal.x;
        goal_msg.pose.pose.position.y = randomGoal.y;
        goal_msg.pose.pose.position.z = 0.0;  // Constant z value and orientations
        goal_msg.pose.pose.orientation.x = 0.0;  
        goal_msg.pose.pose.orientation.y = 0.0;  
        goal_msg.pose.pose.orientation.z = 0.0;  
        goal_msg.pose.pose.orientation.w = 1.0;  

        auto goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        goal_options.result_callback = std::bind(&NavigateToPoseNode::resultCallback, this, std::placeholders::_1);

        if (!client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            return;
        }

        client_->async_send_goal(goal_msg, goal_options);
    }

    void resultCallback(const GoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
        }
    }
};

// Main.cpp
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    GoalGenerator goalGenerator(10, 10);

    // Create the NavigateToPoseNode, passing the GoalGenerator instance
    auto node = std::make_shared<NavigateToPoseNode>("tb1", goalGenerator);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}