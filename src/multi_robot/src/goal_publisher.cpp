#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class NavigateToPoseNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigateToPoseNode(const std::string &namespace_)
        : Node("navigate_to_pose_node_" + namespace_),
          client_(rclcpp_action::create_client<NavigateToPose>(this, "/" + namespace_ + "/navigate_to_pose")),
          goal_handle_future()
    {
        pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/" + namespace_ + "/goal_pose", 10);
        timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&NavigateToPoseNode::sendGoal, this));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_future<GoalHandle::SharedPtr> goal_handle_future;

    void sendGoal()
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = -3.2;
        goal_msg.pose.pose.position.y = 6.20;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.x = 0.0;
        goal_msg.pose.pose.orientation.y = 0.0;
        goal_msg.pose.pose.orientation.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        goal_options.result_callback = std::bind(&NavigateToPoseNode::resultCallback, this, std::placeholders::_1);
        // goal_options.feedback_callback = std::bind(&NavigateToPoseNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.goal_response_callback = std::bind(&NavigateToPoseNode::goalResponseCallback, this, std::placeholders::_1);

        if (!client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            return;
        }

        goal_handle_future = client_->async_send_goal(goal_msg, goal_options);
    }

    void goalResponseCallback(const GoalHandle::SharedPtr &goal_handle)
    {
        if (goal_handle->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)
        {
            RCLCPP_INFO(get_logger(), "Goal accepted by server");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal rejected by server");
        }
    }


    // void feedbackCallback(const GoalHandle::SharedPtr & /*goal_handle*/, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    //   {
    //       RCLCPP_INFO(get_logger(), "Received feedback: %f%%", feedback->distance_travelled * 100.0);
    //   }


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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateToPoseNode>("tb1");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
