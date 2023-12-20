/**
 * @file search.hpp
 * @author Lowell Lobo
 * @author Mayank Deshpande
 * @brief Class Definition for ObjectSearch Class
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

#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

/**
 * @brief Class for performing object detection using a pre-trained model.
 *
 * This class provides functionality to initialize an object detection model,
 * analyze video frames for object detection, and check if a specific object
 * is found in the analyzed frames.
 */
class ObjectSearch {
 public:
  /**
   * @brief Constructor for ObjectSearch class.
   *
   * Initializes an instance of the ObjectSearch class with the provided model
   * and configuration paths.
   *
   * @param modelPath The path to the pre-trained object detection model.
   * @param yolo_names The path to the YOLO class names file.
   */
  ObjectSearch(const std::string& modelPath, const std::string& yolo_names);

  /**
   * @brief Destructor for ObjectSearch class.
   *
   * Cleans up resources and performs necessary actions before destroying
   * an instance of the ObjectSearch class.
   */
  ~ObjectSearch();

  /**
   * @brief Analyzes a video frame for object detection.
   *
   * This function takes a video frame as input and runs object detection
   * using the initialized model.
   *
   * @param frame The input video frame for object detection.
   * @return True if object detection is successful, false otherwise.
   */
  bool runObjectDetection(const cv::Mat& frame);

  /**
   * @brief Checks if the object is found in the analyzed frames.
   *
   * This function returns the status of whether the object of interest
   * is found in the frames analyzed by the object detection model.
   *
   * @return True if the object is found, false otherwise.
   */
  bool isObjectFound() const;

 private:
  /**
   * @brief Private member variable to store the detection status.
   */
  bool objectFound;

  /**
   * @brief Private member variable for the object detection model.
   */
  cv::dnn::Net humanDetectionModel;

  /**
   * @brief Vector to hold all the coco class names for object detection.
   */
  std::vector<std::string> classNames;

};

#endif  // SEARCH_HPP

