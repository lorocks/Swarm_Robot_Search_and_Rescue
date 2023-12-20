# `Swarm Robot Application in Search & Rescue`

A ROS2 package that implements the Search & Rescue (SaR) API for performing Swarm robotic SaR actions simulated in Gazebo.

## Overview
The node calls the search and goal library from the SaR API and using a single launch file all dependencies are launched.
Launch file can be used to spawn n number of robots based on user input.

 ## Nodes
  - NavigateToPoseNode: Publishes navigation points/goals for the robot to move toward and performs object detection while moving
 