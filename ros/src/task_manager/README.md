# Task Manager
This package contains the source code and resources to implement the functionality of the Task Manager node.

Author: Team 1, SSACPS Praktikum 2019

Adapted by: Sergio Quijano and Malte Neuss

## Description
The Task Manager is a local node responsible of keeping the robot’s schedule. The schedule is stored as a list of all the tasks that the robot needs to execute. When the Task allocator notifies the robot that it should take care of a task, the task manager queries the robot’s location to decide in which position in the schedule the new task should be inserted. This behavior favors the execution of the closest task to the current robot’s location, thus providing the robot with a self-adaptation behavior.

The Task manager keeps constant communication with the Task executor node. The manager sends information of the task at the top of the list to the executor, in order for the robot to pursue said task. When the task is completed, the task executor sends a notification to the task manager, and this in turn, notifies to the goal manager that the assigned task is successfully attained. Once a task is completed it, as well as the robot that completed the task, are published on the `all_completed_tasks` topic. Currently, no other node subscribes to this topic and the topic is only used for the purpose of data-analysis.

## ROS Topics
### Publishes to:
* `/goal_attained` [[commons_msgs/Goal.msg](/ros/src/commons_msgs/msg/Goal.msg))]: Publish a message communicating the attainment of a goal.
* `/schedule_updated` [[task_manager_msgs/ScheduleUpdated.msg](/ros/src/task_manager_msgs/msg/ScheduleUpdated.msg))]: Publish the most up to date schedule for this robot.
* `/all_completed_tasks` [[task_manager_msgs/TaskCompleted.msg](/ros/src/task_manager_msgs/msg/TaskCompleted.msg))]: Publishes the position of the task that has been completed and the ID of the robot that completed the task.

### Subscribed to:
* `/confirmation` [[task_allocator_msgs/Confirmation.msg](/ros/src/task_allocator_msgs/msg/Confirmation.msg))]: Listen for confirmation to handle a task.
* `/task_completed` [String]: Listen for info about completed tasks.
* `/amcl_pose` [PoseWithCovarianceStamped]: Listen for info about the current pose of the robot.

### Provide services:
* `/get_schedule` [[task_manager_msgs/GetSchedule.srv](/ros/src/task_manager_msgs/srv/GetSchedule.srv))]: Service provided to other nodes to inform about the current robot's schedule

## Launch parameters
The following parameters need to be provided to initialize the node.  
* `robot_id`: ID of the robot running the current instance of this node
* `initial_x`: x coordinate of the robot's initial position
* `initial_y`: y coordinate of the robot's initial position

These parameters can be edited in the corresponding master launch file (see [test directory](/ros/test)) and, they in turn are passed to the [robot_meta](/ros/src/robot_meta/launch/robot_meta.launch) launch file to start the node.  

