/**
 * @file goals.cpp
 * @author Lowell Lobo
 * @author Mayank Deshpande
 * @brief Class implementation for goal generation for robots
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

#include "goals.hpp"

/**
 * @brief Constructor for GoalGenerator class.
 *
 * Initializes an instance of the GoalGenerator class.
 * Add any necessary initialization code within this function.
 */
GoalGenerator::GoalGenerator() {
  // Add any necessary initialization code
}

/**
 * @brief Destructor for GoalGenerator class.
 *
 * Cleans up resources and performs necessary actions before destroying
 * an instance of the GoalGenerator class.
 */
GoalGenerator::~GoalGenerator() {
  // Add destructor
}

/**
 * @brief Generates goal positions based on the provided map image.
 *
 * This function analyzes the given map image and implements a multi-agent
 * algorithm to generate goal positions for the robots. The generated goal
 * positions are stored in the goalPositions vector.
 *
 * @param mapImage The input map image.
 * @return A vector of GoalPosition representing the generated goal positions.
 */
std::vector<GoalPosition> GoalGenerator::generateGoals(
    const cv::Mat& mapImage) {
  // Implement the multi-agent algorithm to generate goal positions based on the
  // map Update the goalPositions vector accordingly
  GoalPosition testGoal;

  // Placeholder logic
  testGoal.x = 1;
  testGoal.y = 1;

  goalPositions.emplace_back(testGoal);

  // Implement your algorithm to generate goal positions based on the map
  return goalPositions;
}

/**
 * @brief Updates the goal for a specific robot once it reaches its current
 * goal.
 *
 * This function is called when a robot reaches its current goal, and it updates
 * the goal for that specific robot based on its current goal and any other
 * relevant information.
 *
 * @param robotID The identifier of the robot for which the goal needs to be
 * updated.
 * @return The updated GoalPosition for the specified robot.
 */
GoalPosition GoalGenerator::updateGoal(int robotID) {
  // Implement logic to update the goal for a specific robot
  // based on its current goal and any other relevant information
  // Return the updated goal position

  // Placeholder logic
  GoalPosition updatedGoal;

  updatedGoal.x = 1;
  updatedGoal.y = 1;

  return updatedGoal;
}
