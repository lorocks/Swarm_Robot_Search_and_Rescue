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
ObjectSearch::ObjectSearch(const std::string& modelPath,
                           const std::string& configPath)
    : objectFound(false) {
  // Initialize the model and configuration paths
  // Implement code to initialize the object detection model (YOLO or any other)
  // Use modelPath and configPath for loading the YOLO model and configuration
  humanDetectionModel = cv::dnn::readNet(modelPath, configPath);
}

/**
 * @brief Destructor for the ObjectSearch class.
 *
 * Cleans up resources and performs necessary actions before destroying
 * an instance of the ObjectSearch class.
 */
ObjectSearch::~ObjectSearch() {
  // Destructor implementation (if needed)
}


/**
 * @brief Analyzes a video frame for object detection.
 *
 * This function takes a video frame as input and runs object detection
 * using the initialized model. It updates the internal state to indicate
 * whether the object of interest is found in the analyzed frame.
 *
 * @param frame The input video frame for object detection.
 * @return True if object detection is successful, false otherwise.
 */
bool ObjectSearch::runObjectDetection(const cv::Mat& frame) {
  // Implement code to analyze the video stream using the object detection model
  // Update the internal state (e.g., set the boolean to true if the object is
  // found) Return the boolean value indicating whether the object is found or
  // not
  cv::Mat inputBlob;

  // Forward pass the image through the network
  humanDetectionModel.setInput(inputBlob);
  std::vector<cv::Mat> outputs;
  humanDetectionModel.forward(outputs);

  // Process the detection results and update the objectFound variable
  // accordingly
  for (const auto& output : outputs) {
    // For each detection in the output, check if it matches the target object
    // (human)
    //  logic for identifying the object and updating objectFound
  }

  return objectFound;
}

/**
 * @brief Checks if the object is found in the analyzed frames.
 *
 * This function returns the status of whether the object of interest
 * is found in the frames analyzed by the object detection model.
 *
 * @return True if the object is found, false otherwise.
 */
bool ObjectSearch::isObjectFound() const { return objectFound; }
