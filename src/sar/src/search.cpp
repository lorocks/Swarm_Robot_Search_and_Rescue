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
                           const std::string& yolo_names)
    : objectFound(false) {

  // Initialize the YOLOv5 model
  humanDetectionModel = cv::dnn::readNet(modelPath);

  // Create a info stream of the classes from coco.names file and append it to classNames vector
  std::ifstream classNamesFile(yolo_names.c_str());
  std::string text;
  while (classNamesFile >> text) {
    getline(classNamesFile, text);
    classNames.push_back(text);
  }
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
  // Convert the frame to a blob suitable for input to the model
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(416, 416), cv::Scalar(), true, false);

    // Set the input blob for the model
    humanDetectionModel.setInput(blob);

    // Forward pass the blob through the model
    cv::Mat detectionMat = humanDetectionModel.forward();

    // Process the detection results and update the objectFound variable accordingly
    for (int i = 0; i < detectionMat.rows; ++i) {
        float confidence = detectionMat.at<float>(i, 2);

        // Extract class index from detection
        int classId = static_cast<int>(detectionMat.at<float>(i, 1));

        // Check if the detected class is a person
        if (classNames[classId] == "person" && confidence > 0.5) {
            objectFound = true;

            // Extract bounding box coordinates
            int x = static_cast<int>(frame.cols * detectionMat.at<float>(i, 3));
            int y = static_cast<int>(frame.rows * detectionMat.at<float>(i, 4));
            int width = static_cast<int>(frame.cols * (detectionMat.at<float>(i, 5) - detectionMat.at<float>(i, 3)));
            int height = static_cast<int>(frame.rows * (detectionMat.at<float>(i, 6) - detectionMat.at<float>(i, 4)));

            // Draw bounding box
            cv::rectangle(frame, cv::Point(x, y), cv::Point(x + width, y + height), cv::Scalar(0, 255, 0), 2);

            return true;
        }
    }

    // If no person is found in the frame, set objectFound to false
    objectFound = false;
    return false;
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
