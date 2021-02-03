# Dirt Generator
This package contains the source code and resource to implement the functionality of the Dirt Generator node.

Author: Team 2, SSACPS Praktikum 2019

Adapted by: Sergio Quijano and Malte Neuss

## Description
As mentioned in our use case description, dirt is appearing randomly at different locations in the environment. In a real case scenario, the appearance and presence of dirt in the room respond to the dynamics in the environment. Additionally, the deployed robots are unaware of where the dirt is located until they explore the room and discover dirt locations. Therefore, for simulation, it is necessary to have a process that mimics the generation of dirt in the environment.

This global node is responsible for creating random coordinate locations where a dirt will be spawned in the room. When a dirt is spawned in one point in the map, we consider that the containing cell as a dirty tile. The generation process starts by building a list of all possible dirt locations, based on the occupancy map received from the AMCL navigation stack provided by ROS. Using a probability distribution (a beta distribution with parameters α = β = 2), a coordinate pair (x, y) is generated, bounded to the possible dirt locations stored in the list. This coordinate pair is ensured to be free of obstacles around it for the robot to be able to reach this location. Finally, it is validated that the generated coordinate was not already used in a previous iteration of this node.

This node spawns two types of tasks, which are True Positive tasks (tasks that exist) and False Positive tasks (tasks that are physically spawned in the enviornment but represent faulty measurements by the robots). The False Positive tasks are also spawned at regular intervals which is calculated from the False Positive Probability of robot $`i`$ and the spawning interval of True Positive tasks. Both of these are parameters that are provided at launch. If False Positives should be neglected then parameter `false_positives` should be set as False. The spawning interval of False Positives of robot $`i`$ can then be determined as,

$`T_{FP,i} = \frac{1 - p_{FP,i}}{p_{FP,i}} T_{TP}`$ 

where $`T`$ represents the spawning interval and the False Positive probability of robot $`i`$ is defiend as,

$`p_{FP,i} = \frac{n_{FP,i}}{n_{FP,i} + n_{TP}}`$.

In the simulation environment, the generated dirt is inserted as a model with a collision property ([dirt undetected](ros/src/dirt_generator/dirt_object_undetected.sdf) model) that allows for the model to be detected by a robot’s laser scan. Once the dirt is detected, the associated model is updated to disable its collision property ([dirt detected](ros/src/dirt_generator/dirt_object_detected.sdf) model). The generation process is executed every t seconds, given as a parameter when launching the node. The information of the created dirt is published to and managed by the Goal Manager.

## ROS Topics

### Publishes to:
* `/new_dirt` [[goal_manager_msgs/DirtModel.msg](ros/src/goal_manager_msgs/msg/DirtModel.msg))]: Publishes the appearance of new dirt
* `/new_dirt_goalObject` [[goal_manager_msgs/GoalObject.msg](ros/src/goal_manager_msgs/msg/GoalObject.msg))]: Publishes the appearance of a new task as a goal object.
### Subscribed to:
* `/active_tasks` [[goal_manager_msgs/GoalObjectList.msg](ros/src/goal_manager_msgs/msg/GoalObjectList.msg))]: List of all tasks that are currently present in the enviornment (detected and undetected), published by the goal manager. This list is used to avoid generating duplicate random dirt locations.
