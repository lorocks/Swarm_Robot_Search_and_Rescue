#ifndef GOALS_HPP
#define GOALS_HPP

#include <opencv2/opencv.hpp>
#include <vector>

struct GoalPosition {
    double x;
    double y;
};

class GoalGenerator {
public:
    GoalGenerator();
    ~GoalGenerator();

    // Function to generate goal positions based on the map image
    std::vector<GoalPosition> generateGoals(const cv::Mat& mapImage);

    // Function to update the goal for a specific robot once it reaches its current goal
    GoalPosition updateGoal(int robotID);

private:
    // Private members
    std::vector<GoalPosition> goalPositions;
};

#endif // GOALS_HPP
