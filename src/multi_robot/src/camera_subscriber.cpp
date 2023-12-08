/**
 * @file camera_subscriber.cpp
 * @author Lowell Lobo
 * @author Mayank Deshpande
 * @brief ROS2 Node for subscribing to the camera feed of the robots
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

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "search.hpp"
#include "sensor_msgs/msg/image.hpp"

/**
 * @brief A ROS 2 Node for object search using camera images.
 *
 * This class represents a ROS 2 Node that subscribes to a camera feed topic,
 * converts ROS Image messages to OpenCV images, and performs object detection
 * using the ObjectSearch class.
 */
class ObjectSearchNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for ObjectSearchNode class.
   *
   * Initializes the ObjectSearchNode and subscribes to the camera feed topic.
   */
  ObjectSearchNode();

 private:
  /**
   * @brief Callback function for the camera feed subscription.
   *
   * This function is called whenever a new camera image message is received.
   *
   * @param msg The received sensor_msgs::msg::Image message.
   */
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief ObjectSearch instance for object detection.
   *
   * An instance of the ObjectSearch class is initialized for performing
   * object detection using a specific model and configuration.
   */
  std::shared_ptr<ObjectSearch> objectSearch{
      std::make_shared<ObjectSearch>("model/path", "model/path")};

  /**
   * @brief Subscription handle for the camera feed topic.
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSubscription;
};

/**
 * @brief Constructor for ObjectSearchNode class.
 *
 * Initializes the ObjectSearchNode and subscribes to the camera feed topic.
 */
ObjectSearchNode::ObjectSearchNode() : Node("object_search_node") {
  // Subscribe to the camera feed topic
  cameraSubscription = this->create_subscription<sensor_msgs::msg::Image>(
      "robot_i/camera_topic", 10,
      std::bind(&ObjectSearchNode::cameraCallback, this,
                std::placeholders::_1));
}

/**
 * @brief Callback function for the camera feed subscription.
 *
 * This function is called whenever a new camera image message is received.
 *
 * @param msg The received sensor_msgs::msg::Image message.
 */
void ObjectSearchNode::cameraCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  // Convert ROS Image message to OpenCV image
  // TODO(Mayank): Add code to convert ROS Image to OpenCV image

  // Run object detection using the ObjectSearch class
  // TODO(Mayank): Add code to run object detection using ObjectSearch class

  // Check if the object is found
  // TODO(Mayank): Add code to check if the object is found and perform
  // necessary actions
}

/**
 * @brief Main function for the object search node.
 *
 * Initializes ROS 2, spins the ObjectSearchNode, and shuts down ROS 2 upon
 * completion.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return The exit code.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectSearchNode>());
  rclcpp::shutdown();
  return 0;
}
