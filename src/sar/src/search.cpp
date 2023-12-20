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

#include <opencv2/core.hpp>

/**
 * @brief Constructor for the ObjectSearch class.
 *
 * Initializes an instance of the ObjectSearch class with the provided model
 * and configuration paths.
 *
 * @param modelPath The path to the pre-trained object detection model.
 * @param yolo_names The path to the YOLO class names file.
 */
ObjectSearch::ObjectSearch(const std::string& modelPath,
                           const std::string& yolo_names)
    : objectFound(false), humanDetectionModel(cv::dnn::readNet(modelPath)) {
  // Initialize the YOLOv5 model
  // humanDetectionModel = cv::dnn::readNet(modelPath);

  // Create a info stream of the classes from coco.names file and append it to
  // classNames vector
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
  cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(640, 640),
                                        cv::Scalar(), true, false);

  // Set the input blob for the model
  humanDetectionModel.setInput(blob);

  std::vector<cv::Mat> outputs;

  // Forward pass the blob through the model
  humanDetectionModel.forward(outputs, humanDetectionModel.getUnconnectedOutLayersNames());

  float* data = reinterpret_cast<float*>(outputs[0].data);

  for (int i = 0; i < 25200; ++i) {
    float confidence = data[4];
    if (confidence > 0.5) {
      float* classes_scores = data + 5;
      cv::Mat scores(1, classNames.size(), CV_32FC1, classes_scores);
      cv::Point class_id;
      double max_score;
      cv::minMaxLoc(scores, 0, &max_score, 0, &class_id);
      
      // Check if the detected object is a "bowl" 
      if (max_score > 0.5 && class_id.x == 0) {
        objectFound = true;
        return true;
      }
    }
    data += 85;
  }

  // If no bowl is found in the frame, set objectFound to false
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
bool ObjectSearch::isObjectFound() const {
  // Check if the class label for a "bowl" is present in the classNames vector
  auto it = std::find(classNames.begin(), classNames.end(), "bowl");
  
  // If the class label for a "bowl" is found and the object is found in the frames, return true
  return (it != classNames.end() && objectFound);
}

