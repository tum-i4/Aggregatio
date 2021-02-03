# Bidder
This package contains the source code and resources to implement the functionality of the Bidder node.

Author: Team 1, SSACPS Praktikum 2019

Adapted by: Sergio Quijano

## Description
The Bidder node is executed per robot and is in charge of receiving the new task notifications from the Goal Manager. Once a new notification is received, the robot needs to calculate the cost that it may incur by carrying the task out. The coordinates of tasks assigned to the robot are stored in a list, which represent the route it has to follow. This stored list is called the robot’s schedule. Upon a new notification, the robot checks its current route to decide which task is the closest to the newly appeared dirt. Subsequently the robot calculate the additional costs for adding the new dirt to its route before and after the closest point. The robot compares the calculated costs for adding the new task before and after the closest point and the smaller cost is used to propose a new bid to the Task Allocator. A visualization of this procedure is depicted in following figure.


<img src="ros/documentation_resources/cost_calculation.png" alt="Task's cost calculation" width="697" height="348" /> 

For cost calculation, the node uses the `move_base` service named `make_plan` to receive the actual trajectories for all possible new paths (as shown in the figure above) in accordance to the costmap. For these trajectories the costs per traversed grid cell as well the length of the path are summed up to build the cost for inserting the new task to the current schedule. Of these two costs, the smaller one is published, alongside the robot’s ID, to the Task Allocator node to decide to which robot the task will be assigned. The cost calculation is executed as many times as tasks notifications are received in this node.

As an alternative to the calculation process described above, known as adaptive schedule, a more simplistic approach can be used. In such approach, the length of the schedule is considered as the cost for the new task, and it is added at the end of the robot's schedule.

## ROS Topics
### Publishes to:
* `/goal_attained` [[bidder_msgs/Bid.msg](ros/src/bidder_msgs/msg/Bid.msg))]: Publish a bid for a received goal to pursue.

### Subscribed to:
* `/new_goal` [[commons_msgs/Goal.msg](ros/src/commons_msgs/msg/Goal.msg))]: Listen for new goals to be pursued.
* `/move_base/global_costmap/costmap` [nav_msgs.msg/OccupancyGrid]: Listen for the costmap published by the move_base package of the ROS navigation stack.

## Launch parameters
The following parameters need to be provided to initialize the node.  
* `robot_id`: ID of the robot running the current instance of this node
* `adaptive_scheduling`: Flag indicating if adaptive scheduling should be used

These parameters can be provided/edited in the corresponding master launch file (see [test directory](ros/test)) and, they in turn are passed to the [robot_meta](ros/src/robot_meta/launch/robot_meta.launch) launch file to start the node.

