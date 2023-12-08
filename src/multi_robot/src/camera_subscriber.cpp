#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "search.hpp"  // Include your ObjectSearch class

class ObjectSearchNode : public rclcpp::Node {
public:
    ObjectSearchNode();

private:
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // Correct declaration and initialization of objectSearch
    std::shared_ptr<ObjectSearch> objectSearch{
        std::make_shared<ObjectSearch>("/path/to/the/model", "/path/to/the/config")};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cameraSubscription;
};

ObjectSearchNode::ObjectSearchNode() : Node("object_search_node") {
    // Subscribe to the camera feed topic
    cameraSubscription = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera_topic",  // Replace with your actual camera topic
        10,  // Adjust the queue size as needed
        std::bind(&ObjectSearchNode::cameraCallback, this, std::placeholders::_1)
    );
}

void ObjectSearchNode::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS Image message to OpenCV image
    // TODO: Add code to convert ROS Image to OpenCV image

    // Run object detection using the ObjectSearch class
    // TODO: Add code to run object detection using ObjectSearch class

    // Check if the object is found
    // TODO: Add code to check if the object is found and perform necessary actions
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSearchNode>());
    rclcpp::shutdown();
    return 0;
}
