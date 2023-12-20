#include <gtest/gtest.h>

#include <cmath>

#include "goals.hpp"
#include "search.hpp"

TEST(unit_test_object_detect_ball, this_should_pass) {
  ObjectSearch obj("../../../src/sar/models/yolov5s.onnx",
                   "../../../src/sar/models/coco.names");

  // Load a static image using cv::imread("../../assets/image.jpg"); and pass it
  // through the runObjectDetection method
  cv::Mat image = cv::imread("../../../src/sar/assets/Main.jpg");

  bool val = obj.runObjectDetection(image);

  EXPECT_FALSE(val);
}

// TEST(unit_test_object_detect_people, this_should_pass) {
//   ObjectSearch obj("../../../src/sar/models/yolov5s.onnx",
//                    "../../../src/sar/models/coco.names");

//   // Load a static image using cv::imread("../../assets/image.jpg"); and pass it
//   // through the runObjectDetection method
//   cv::Mat image = cv::imread("../../../src/sar/assets/people.jpg");

//   bool val = obj.runObjectDetection(image);

//   EXPECT_FALSE(val);
// }

TEST(unit_test_object_detect_cycle, this_should_pass) {
  ObjectSearch obj("../../../src/sar/models/yolov5s.onnx",
                   "../../../src/sar/models/coco.names");

  // Load a static image using cv::imread("../../assets/image.jpg"); and pass it
  // through the runObjectDetection method
  cv::Mat image = cv::imread("../../../src/sar/assets/bike.jpg");

  bool val = obj.runObjectDetection(image);

  EXPECT_FALSE(val);
}

TEST(unit_test_check_object_found, this_should_pass) {
  ObjectSearch obj("../../../src/sar/models/yolov5s.onnx",
                   "../../../src/sar/models/coco.names");

  bool val = obj.isObjectFound();

  EXPECT_FALSE(val);
}

TEST(unit_test_generate_goals, this_should_pass) {
  GoalGenerator obj(1, 1);

  auto positions = obj.generateRandomGoal();

  EXPECT_GT((positions.x * positions.x) + (positions.y * positions.y), 0);
}
