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
 * @brief Class for generating random goals within the map boundaries.
 */
class GoalGenerator {
public:
    /**
     * @brief Constructor for GoalGenerator class.
     */
    GoalGenerator(int mapWidth, int mapHeight);

    /**
     * @brief Generates a random goal position within the map boundaries.
     *
     * @return A GoalPosition representing the randomly generated goal position.
     */
    GoalPosition generateRandomGoal();

private:
    /**
     * @brief Paramter to hold the width of the map
     * 
     */
    int mapWidth_;

    /**
     * @brief Paramter to hold the height of the map
     * 
     */
    int mapHeight_;
};

#endif  // GOALS_HPP
