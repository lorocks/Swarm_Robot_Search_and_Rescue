#include <gtest/gtest.h>

#include <cmath>

#include "goals.hpp"
#include "search.hpp"

TEST(unit_test_object_detect, this_should_pass) {
  ObjectSearch obj("../../models/modelfile", "../../models/configfile");

  // Load a static image using cv::imread("../../assets/image.jpg"); and pass it
  // through the runObjectDetection method
  cv::Mat image;

  bool val = obj.runObjectDetection(image);

  EXPECT_TRUE(val);
}

TEST(unit_test_check_object_found, this_should_pass) {
  ObjectSearch obj("../../models/modelfile", "../../models/configfile");

  bool val = obj.isObjectFound();

  EXPECT_FALSE(val);
}

// TEST(unit_test_generate_goals, this_should_pass) {
//   GoalGenerator obj;

//   cv::Mat mapImage;

//   auto positions = obj.generateGoals(mapImage);

//   EXPECT_GT(positions.size(), 0);
// }

// TEST(unit_test_goal_update, this_should_pass) {
//   GoalGenerator obj;

//   auto goal = obj.updateGoal(0);

//   auto goalVector = sqrt((goal.x * goal.x) + (goal.y * goal.y));

//   EXPECT_GT(goalVector, 0);
// }
