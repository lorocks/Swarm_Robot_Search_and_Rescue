# Swarm Robot Search & Rescue

[![codecov](https://codecov.io/gh/TommyChangUMD/ENPM808X-final-project-boilerplate/branch/main/graph/badge.svg?token=KRAHD3BZP7)](https://codecov.io/gh/TommyChangUMD/ENPM808X-final-project-boilerplate)

![CICD Workflow status](https://github.com/TommyChangUMD/ENPM808X-final-project-boilerplate/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)


## Packages
 - sar: A C++ API used for navigation and object detection with robotd
 - multi_robot: A package created to use the sar API for Search & Rescue operations and Swarm implementation
  
## How to generate package dependency graph

``` bash
colcon graph --dot | dot -Tpng -o depGraph.png
open depGraph.png
```
[<img src=screenshots/depGraph.png
    width="20%" 
    style="display: block; margin: 0 auto"
    />](screenshots/depGraph.png)



## How to build

```bash
rm -rf build/ install/
colcon build 
source install/setup.bash
```

## How to build for tests (unit test and integration test)

```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```

## How to run tests (unit and integration)

```bash
source install/setup.bash
colcon test
# View test results
colcon test-result --all --verbose
```

## How to generate coverage reports after running colcon test

First make sure we have run the unit test already.

```bash
colcon test
```

### Test coverage report for `multi_robot`:

``` bash
ros2 run multi_robot generate_coverage_report.bash
open build/multi_robot/test_coverage/index.html
```

### Test coverage report for `sar`:

``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select sar \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RAN=1
open build/sar/test_coverage/index.html
```

### combined test coverage report

``` bash
./do-tests.bash
```

## How to generate project documentation
``` bash
./do-docs.bash
```


 # Run nodes
 Functionality is stil being built
