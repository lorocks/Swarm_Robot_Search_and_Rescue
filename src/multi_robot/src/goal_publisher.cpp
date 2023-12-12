// Copyright 2016 Open Source Robotics Foundation, Inc.

//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


/**
 * @brief MinimalPublisher class that inherits form the Node class in rclcpp
 * used to built a ros2 node
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *  Create a publisher and publish messages to the topic "/topic" at 500ms
   * intervals
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    this->declare_parameter("pub_frequency", 750);
    int para_freq = this->get_parameter("pub_frequency").as_int();
    if (para_freq < 450) {
      RCLCPP_FATAL(this->get_logger(),
                   "Publish time too fast...\n Selecting 750ms");
      frequency = 750;
    } else if (para_freq > 3000) {
      RCLCPP_ERROR(this->get_logger(), "Publish time not optimal");
      frequency = para_freq;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Setting custom publish frequency");
      frequency = para_freq;
    }
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(frequency),
        std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  /**
   * @brief A member function that runs based on set timer
   *
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Lowell's message number " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing message: '%s'",
                message.data.c_str());
    publisher_->publish(message);
  }


  /**
   * @brief Create a timer shared pointer from rclcpp to be used in
   * implementation
   *
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Create a publisher shared pointer from rclcpp to be used in the
   * implementation
   *
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  /**
   * @brief Create a count variable to increment the message number in published
   * message
   *
   */
  size_t count_;

  /**
   * @brief Create a frenquency variable from talker publish frequency
   *
   */
  int frequency;
};

/**
 * @brief The main implementation of the class
 *
 * @param argc Console input argument
 * @param argv Console input argument
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
