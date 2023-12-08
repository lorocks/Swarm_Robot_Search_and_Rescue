/**
 * @file goals.hpp
 * @author Lowell Lobo
 * @author Mayank Deshpande
 * @brief Class Definition for GoalGenerator Class
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

#ifndef GOALS_HPP
#define GOALS_HPP

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief Structure representing a goal position with x and y coordinates.
 */
struct GoalPosition {
  double x; /**< X-coordinate of the goal position. */
  double y; /**< Y-coordinate of the goal position. */
};

/**
 * @brief Class for generating and managing goals based on a map image.
 */
class GoalGenerator {
 public:
  /**
   * @brief Constructor for GoalGenerator class.
   */
  GoalGenerator();

  /**
   * @brief Destructor for GoalGenerator class.
   */
  ~GoalGenerator();

  /**
   * @brief Generates goal positions based on the provided map image.
   *
   * This function analyzes the map image and generates goal positions.
   *
   * @param mapImage The input map image.
   * @return A vector of GoalPosition representing the generated goal positions.
   */
  std::vector<GoalPosition> generateGoals(const cv::Mat& mapImage);

  /**
   * @brief Updates the goal for a specific robot once it reaches its current
   * goal.
   *
   * This function is called when a robot reaches its current goal, and it
   * updates the goal for that specific robot.
   *
   * @param robotID The identifier of the robot for which the goal needs to be
   * updated.
   * @return The updated GoalPosition for the specified robot.
   */
  GoalPosition updateGoal(int robotID);

 private:
  /**
   * @brief Private member variable to store the generated goal positions.
   */
  std::vector<GoalPosition> goalPositions;
};

#endif  // GOALS_HPP
