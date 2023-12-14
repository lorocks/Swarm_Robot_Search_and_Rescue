/**
 * @file search.hpp
 * @author Lowell Lobo
 * @author Mayank Deshpande
 * @brief Class Definition for RobotDesc Class
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
#include "goals.hpp"
#include "search.hpp"

/**
 * @brief Class for defining the Robot's decription and calling the ObjectSearch and GoalPosition classes.
 *
 * This class unifies the two classes into one class call
 */
class RobotDesc {
 public:
  /**
   * @brief Constructor for RobotDesc class.
   *
   * Initializes an instance of the RobotDesc class with the provided model
   * and configuration paths.
   *
   * @param modelPath The path to the pre-trained object detection model.
   * @param configPath The path to the configuration file for the model.
   */
  RobotDesc(const std::string& modelPath, const std::string& configPath);

  /**
   * @brief Destructor for RobotDesc class.
   *
   * Cleans up resources and performs necessary actions before destroying
   * an instance of the RobotDesc class.
   */
  ~RobotDesc();

 private:
  /**
   * @brief Private member variable to store the robot number.
   */
  int robotNum;

  /**
   * @brief Private member variable for holding object of detection model.
   */
  ObjectSearch detection;

  /**
   * @brief Private member variable for holding object of navigation model.
   */
  GoalPosition navigate;
};

#endif  // SEARCH_HPP
