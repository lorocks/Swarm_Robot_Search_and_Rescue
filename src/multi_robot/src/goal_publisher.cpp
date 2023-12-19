#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp> 
#include <rclcpp_action/rclcpp_action.hpp>
#include "goals.hpp"
#include "search.hpp"

class NavigateToPoseNode : public rclcpp::Node
{
public:
    NavigateToPoseNode(const std::string& robot_namespace)
        : Node("navigate_to_pose_client_" + robot_namespace), present_goal(0), robot_namespace_(robot_namespace),
          ObjectSearch("path/to/your/model.onnx", "path/to/your/coco.names")
    {
        // Create an ActionClient for the NavigateToPose action
        goal_send = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/" + robot_namespace + "/navigate_to_pose");

        image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/" + robot_namespace + "/camera/image_raw",
            10,
            std::bind(&NavigateToPoseNode::imageCallback, this, std::placeholders::_1));

        // Wait for the action server to be available
        if (!goal_send->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available for robot %s", robot_namespace.c_str());
            return;
        }

        // Create a list of GoalPositions for each robot
        if (robot_namespace == "tb1")
        {
            // goals_list = generateGoalListRobot1();
            goals_list = {
                {0.5, 1.8},
                {0.21, -1.8},
                {0.5, 1.8},
                {0.21, -1.8},
                {0.5, 1.8}
            };
        }
        else if (robot_namespace == "tb2")
        {
            // goals_list = generateGoalListRobot2();
            goals_list = {
                {1.87, 0.902},
                {-0.87, 2.1},
                {1.87, 0.902},
                {-0.87, 2.1},
                {1.87, 0.902}
            };
        }

        // Send the first goal
        sendNextGoal();
    }

private:
    void sendNextGoal()
    {
        if (present_goal < goals_list.size())
        {
            // Create a goal for the NavigateToPose action with constant z and orientation values
            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.pose.position.x = goals_list[present_goal].x;
            goal_msg.pose.pose.position.y = goals_list[present_goal].y;
            goal_msg.pose.pose.position.z = 0.0;  // Constant z value
            goal_msg.pose.pose.orientation.x = 0.0;
            goal_msg.pose.pose.orientation.y = 0.0;
            goal_msg.pose.pose.orientation.z = 0.0;
            goal_msg.pose.pose.orientation.w = 1.0000000;

            // Send the goal to the action server
            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&NavigateToPoseNode::result_callback, this, std::placeholders::_1);
            auto future_goal_handle = goal_send->async_send_goal(goal_msg, send_goal_options);

            RCLCPP_INFO(get_logger(), "[%s] Sending goal %zu", robot_namespace_.c_str(), present_goal);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "[%s] All goals achieved!", robot_namespace_.c_str());
        }
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "[%s] Goal %zu succeeded!", robot_namespace_.c_str(), present_goal);

            // Move to the next goal
            present_goal++;
            sendNextGoal();
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "[%s] Goal %zu was aborted", robot_namespace_.c_str(), present_goal);

            // Retry the same goal on failure
            sendNextGoal();
            break;
        default:
            RCLCPP_ERROR(get_logger(), "[%s] Unknown result code", robot_namespace_.c_str());

            present_goal++;
            sendNextGoal();
            break;
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the ROS image from the robots to the OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "CV Bridge Exception: %s", e.what());
            return;
        }

        // Run the object detection on the camera
        if (ObjectSearch.runObjectDetection(cv_ptr->image))
        {
            // Object found, cancel goals for both robots
            RCLCPP_INFO(get_logger(), "Object found! Cancelling goals for %s", robot_namespace_.c_str());
            future_goal_handle->cancel_goal();
        }
    }

    std::vector<GoalPosition> generateGoalListRobot1()
    {
        // Create a list of GoalPositions with random values for robot 1
        std::vector<GoalPosition> goals;
        GoalGenerator goal_generator(1, 1);
        for (int i = 0; i < 5; ++i) // i is the number of goals to be generated
        {
            goals.push_back(goal_generator.generateRandomGoal());
        }
        return goals;
    }

    std::vector<GoalPosition> generateGoalListRobot2()
    {
        // Create a list of GoalPositions with random values for robot 2
        std::vector<GoalPosition> goals;
        GoalGenerator goal_generator(1, 1); 
        for (int i = 0; i < 5; ++i) // i is the number of goals to be generated
        {
            goals.push_back(goal_generator.generateRandomGoal());
        }
        return goals;
    }

    std::vector<GoalPosition> goals_list;
    size_t present_goal;
    std::string robot_namespace_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr goal_send;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future_goal_handle;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create an instance of NavigateToPoseNode with the provided robot namespace
    auto node = std::make_shared<NavigateToPoseNode>(argv[1]);

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


