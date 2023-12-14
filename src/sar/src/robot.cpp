/**
 * @file search.cpp
 * @author Lowell Lobo
 * @author Mayank Deshpande
 * @brief Class Implementation for ObjectSearch Class
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

#include "search.hpp"

/**
 * @brief Constructor for the ObjectSearch class.
 *
 * Initializes an instance of the ObjectSearch class with the provided model
 * and configuration paths.
 *
 * @param modelPath The path to the pre-trained object detection model.
 * @param configPath The path to the configuration file for the model.
 */
RobotDesc::RobotDesc(const std::string& modelPath,
                           const std::string& configPath)
    : detection(modelPath, configPath) {
  // Initialize the model and configuration paths
  // Implement code to initialize the object detection model (YOLO or any other)
  // Use modelPath and configPath for loading the YOLO model and configuration
  // humanDetectionModel = cv::dnn::readNet(modelPath, configPath);
}

/**
 * @brief Destructor for the ObjectSearch class.
 *
 * Cleans up resources and performs necessary actions before destroying
 * an instance of the ObjectSearch class.
 */
RobotDesc::~RobotDesc() {
  // Destructor implementation (if needed)
}
