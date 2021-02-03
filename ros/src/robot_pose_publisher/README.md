# Pose publisher
This package contains the source code to implement the functionality of the Pose Publisher helper node.

Author: Sergio Quijano

## Description
The Pose publisher helper node aims to provide a list with all robots' poses during the simulation. The list is updated every 30 seconds. 

## ROS Topics
### Publishes to:
* `/robots_locations` [[robot_pose_publisher/RobotLocations.msg](/ros/src/robot_pose_publisher/msg/RobotLocations.msg))]: Publish a list with the poses of all robots in the simulation.

### Subscribed to:
* `/amcl_pose` [PoseWithCovarianceStamped]: Listen for info about the current AMCL pose of each robot in the simulation.