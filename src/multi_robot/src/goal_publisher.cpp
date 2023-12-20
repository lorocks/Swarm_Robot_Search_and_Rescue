/**
 * @file goal_publisher.cpp
 * @author Lowell Lobo
 * @author Mayank Deshpande
 * @brief ROS2 Implementation using Search and Rescue API
 * @version 0.1
 * @date 2023-12-08
 *
 * @copyright Copyright © 2023 <copyright holders>
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions: The above copyright
 * notice and this permission notice shall be included in all copies or
 * substantial portions of the Software. THE SOFTWARE IS PROVIDED “AS IS”,
 * WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
 * THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 */
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "goals.hpp"
#include "search.hpp"

/**
 * @brief ROS2 Class inheriting from rclcpp Node to implment a node that uses
 * the SaR API for Swarm Robotics
 *
 */
class NavigateToPoseNode : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Navigate To Pose Node object
   *
   * @param robot_namespace
   */
  explicit NavigateToPoseNode(const std::string& robot_namespace)
      : Node("navigate_to_pose_client_" + robot_namespace),
        present_goal(0),
        robot_namespace_(robot_namespace),
        find("src/multi_robot/models/yolov5s.onnx",
             "src/multi_robot/models/coco.names") {
    // Create an ActionClient for the NavigateToPose action
    goal_send = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "/" + robot_namespace + "/navigate_to_pose");

    // Create a subscriber to read camera streaming data
    image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/" + robot_namespace + "/camera/image_raw", 10,
        std::bind(&NavigateToPoseNode::imageCallback, this,
                  std::placeholders::_1));

    // Wait for the action server to be available
    if (!goal_send->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available for robot %s",
                   robot_namespace.c_str());
      return;
    }

    goals_list = generateGoalListRobot1();
    // Create a list of GoalPositions for each robot
    if (robot_namespace == "tb1") {
      // goals_list = generateGoalListRobot1();
      goals_list = {
          {0.5, 1.8}, {0.21, -1.8}, {0.5, 1.8}, {0.21, -1.8}, {0.5, 1.8}};
    } else if (robot_namespace == "tb2") {
      goals_list = {{1.87, 0.902},
                    {-0.87, 2.1},
                    {1.87, 0.902},
                    {-0.87, 2.1},
                    {1.87, 0.902}};
    }

    // Send the first goal
    sendNextGoal();
  }

 private:
  /**
   * @brief Method to push goal generated into the Nav2 stack for autonomous
   * navigation
   *
   */
  void sendNextGoal() {
    if (present_goal < goals_list.size()) {
      // Create a goal for the NavigateToPose action with constant z and
      // orientation values
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
      auto send_goal_options = rclcpp_action::Client<
          nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(
          &NavigateToPoseNode::result_callback, this, std::placeholders::_1);
      auto future_goal_handle =
          goal_send->async_send_goal(goal_msg, send_goal_options);

      RCLCPP_INFO(get_logger(), "[%s] Sending goal %zu",
                  robot_namespace_.c_str(), present_goal);
    } else {
      RCLCPP_INFO(get_logger(), "[%s] All goals achieved!",
                  robot_namespace_.c_str());
    }
  }

  /**
   * @brief Async callback for the nav2 action of posting goal position
   *
   * Returns response about navigation based on current goal sent
   *
   * @param result Parameter to check callback and action status
   */
  void result_callback(
      const rclcpp_action::ClientGoalHandle<
          nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "[%s] Goal %zu succeeded!",
                    robot_namespace_.c_str(), present_goal);

        // Move to the next goal
        present_goal++;
        sendNextGoal();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "[%s] Goal %zu was aborted",
                     robot_namespace_.c_str(), present_goal);

        // Retry the same goal on failure
        sendNextGoal();
        break;
      default:
        RCLCPP_ERROR(get_logger(), "[%s] Unknown result code",
                     robot_namespace_.c_str());

        present_goal++;
        sendNextGoal();
        break;
    }
  }

  /**
   * @brief Callback to call object detection for the camera topic subscriber
   *
   * @param msg Parameter holding the image data
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert the ROS image from the robots to the OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "CV Bridge Exception: %s", e.what());
      return;
    }

    // Run the object detection on the camera
    if (find.runObjectDetection(cv_ptr->image)) {
      // Object found, cancel goals for both robots
      RCLCPP_INFO(get_logger(), "Object found! Cancelling goals for %s",
                  robot_namespace_.c_str());
      goal_send->async_cancel_all_goals();
    }
  }

  /**
   * @brief Method 1 to generate a list of goals by storing values into a vector
   *
   * @return std::vector<GoalPosition> : return vector with goal positions
   */
  std::vector<GoalPosition> generateGoalListRobot1() {
    // Create a list of GoalPositions with random values for robot 1
    std::vector<GoalPosition> goals;
    GoalGenerator goal_generator(1, 1);
    for (int i = 0; i < 5; ++i) {  // i is the number of goals to be generated
      goals.push_back(goal_generator.generateRandomGoal());
    }
    return goals;
  }

  /**
   * @brief vector of struct GoalPostion to hold a list of goal values for
   * navigation
   *
   */
  std::vector<GoalPosition> goals_list;

  /**
   * @brief Counter for current goal iteration
   *
   */
  size_t present_goal;

  /**
   * @brief Parameter to set the namespace for the node
   *
   */
  std::string robot_namespace_;

  /**
   * @brief rclcpp action object for sending goal position to Nav2
   *
   */
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr goal_send;

  /**
   * @brief rclcpp action object for handling the client action asynchronously
   *
   */
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
      future_goal_handle;

  /**
   * @brief Object for subscribing to camera topic to get video streaming
   * information
   *
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  /**
   * @brief Object of class ObjectSearch used for object detection
   *
   */
  ObjectSearch find;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Create an instance of NavigateToPoseNode with the provided robot namespace
  auto node = std::make_shared<NavigateToPoseNode>(argv[1]);

  auto spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  spinner->add_node(node);

  // Run the spinner in a separate thread
  std::thread spinner_thread([spinner]() { spinner->spin(); });

  // Wait for the spinner to finish
  spinner_thread.join();

  rclcpp::shutdown();

  return 0;
}
