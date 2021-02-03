# Task Executor
This package contains the source code and resources to implement the functionality of the Task Allocator node.

Author: Team 1, SSACPS Praktikum 2019

Adapted by: Sergio Quijano

## Description
The Task Executor node is in charge of communicating with the navigation stack in order to move the robot towards the location of the task that it needs to complete. As soon as the Task Manager notifies the executor of a new task, the executor sends a message to the `move_base` component in the navigation stack, initiating the robot’s trajectory towards its goal. The navigation stack will take care of all the robot’s movements to reach the assigned goal and once the goal is reached, it notifies to the task executor. In turn, the task executor send a message to the task manager notifying the task completion.

An additional feature in the task executor is the execution of so-called virtual tasks. When the robot’s schedule is empty, the task executor selects a location from a predefined list of locations and sends it as a new goal to the navigation stack. This feature allows the robot to keep moving in the environment, enabling the detection of new dirt tiles in the context.

## ROS Topics
### Publishes to:
* `/task_completed` [String]: Publish a message communicating the completion of a task.

### Subscribed to:
* `/schedule_updated` [[task_manager_msgs/ScheduleUpdated.msg](/ros/src/task_manager_msgs/msg/ScheduleUpdated.msg))]: Listen for info of the most up to date schedule of the robot.

