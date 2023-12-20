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

#include <random>

/**
 * @brief Construct a new Goal Generator:: Goal Generator object
 *
 * Instantiates a object of GoalGenerator Class and set the class parameters
 *
 * @param mapWidth The width of the map
 * @param mapHeight The height of the map
 */
GoalGenerator::GoalGenerator(int mapWidth, int mapHeight)
    : mapWidth_(mapWidth), mapHeight_(mapHeight) {}

/**
 * @brief Generates random goal positions for navigation and traversal
 *
 * Uses the std methods for generating uniformly distributed random number
 * between the range of the map's width and height
 *
 * @return GoalPosition struct return type
 */
GoalPosition GoalGenerator::generateRandomGoal() {
  // Define a object of struct for returning value
  GoalPosition goal;

  // Use a random number generator to generate random x and y coordinates within
  // the map
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> x_dist(0.0, static_cast<double>(mapWidth_));
  std::uniform_real_distribution<> y_dist(0.0, static_cast<double>(mapHeight_));

  // Update (x, y) values for return
  goal.x = x_dist(gen);
  goal.y = y_dist(gen);

  return goal;
}
