# `Search & Rescue (SaR)`

## Overview

A stand-alone C++ API using:

- CMake
- GoogleTest
- OpenCV

The API is primarily built to be used with ROS2 but the API does not explicitly depend on ROS2 or any of its packages.
It is a stand-alone API for Search and Rescue operations for robots by navigating and object detection in a provided location and cost map.

The API consistes of two libraries
- goals: A navigation library used to dynamically compute locations in a provided map where robots should move toward.
- search: A object detection library used to check for pre-defined objects in the scene.

### Library - goals
It consists of the main method,
- generateRandomGoal(): is used to generate random positions for the robot to navigate toward

Library can be added using by including "goals.hpp"


### Library - search
It consists of two main method,
- runObjectDetection(): is used to pass the scene image for object detection and return the outputs from analysis
- isObjectFound(): is used to set a global value for whether an object is found or not and by which robot

Library can be added using by including "search.hpp"

## Standard install via command-line
```bash
# Configure the project and generate a native build system:
  # Re-run this command whenever any files have been changed.
  colcon build --packages-select sar
  # Or build all packages using
  colcon build
```

## Running unit tests
```bash
# Run unit tests using the command
  colcon test
# View test results
  colcon test-result --all
# View verbose test results
  colcon test-result --all --verbose
```
 
## Building for code coverage

```bash
# Set the build type to Debug and WANT_COVERAGE=ON
  cmake -D COVERAGE=ON -S ./ -B build/
# Now, do a clean compile, run unit test, and generate the covereage report
  cmake --build build/ --clean-first --target all test_coverage
# open a web browser to browse the test coverage report
  open build/test_coverage/index.html

This generates a index.html page in the build/test_coverage sub-directory that can be viewed locally in a web browser.
```
