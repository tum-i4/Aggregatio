# Task Allocator
This package contains the source code and resources to implement the functionality of the Task Allocator node.

Author: Team 1, SSACPS Praktikum 2019

Adapted by: Sergio Quijano

## Description
This node is responsible for collecting the bids that the participating robots issue about a same task. The allocator keeps record of all the collaborating robots in oder to decide when to assign a task. Since the subscription works sequentially, all proposals of a task have to be stored in a list, until the set of proposals is complete for the number of robots active in the room. A timer function triggers a decision after a chosen amount of seconds. Only after all robots have issued a bid or after the defined timeout, the allocator compares the received costs and assigns the task to the bidder of the lowest cost. When the allocator has made its decision, all robots are notified which robot should add the task to its schedule.

## ROS Topics

### Publishes to:
* `/confirmation` [[task_allocator_msgs/Confirmation.msg](/ros/src/task_allocator_msgs/msg/Confirmation.msg))]: Publish a confirmation message for the robot with the best bid to handle the task.

### Subscribed to:
* `/bid` [[bidder_msgs/Confirmation.msg](/ros/src/bidder_msgs/msg/Bid.msg))]: Listen for info the bids of the participating robots.

## Launch parameters
The following parameters need to be provided to initialize the node.  
* `no_of_robots`: Number of robots in the current simulation. After the nodes initialization, this value will be updated according to the number of bidder detected. Default: 2 robots
* `timeout_sec`: Time (in seconds) to wait for robots' bids. Default: 3s
* `refresh_robot_number_interval`: Time (in seconds) to wait to refresh the number of robots. Default: 60s