# Goal Manager and Knowledge Aggregator
This package contains the source code and resources to implement the functionality of the Goal Manager and Knowledge aggregator nodes.

As depicted in the high-level architecture of our system, the Goal Manager and Knowledge Aggregator are two different and separate nodes. Originally, the Goal Manager was meant to call a service of the Knowledge Aggregator to get the associated subjective opinion for a point being handled by the Goal Manager.  However, while performing the tests of these nodes, we encountered a critical exception that made the Knowledge aggregator crash when the corresponding service was being handled. After some debugging and despite some modifications, we decided to merge the functionality of these nodes into the code of the Goal Manager. This way, we avoided the service call causing the problem.

In the following, although for ROS this package is treated as a single node, we describe the implemented functionality as two separate nodes.

# Goal Manager
Author: Team 2, SSACPS Praktikum 2019

Adapted by: Sergio Quijano and Malte Neuss

## Description
This node is responsible for managing the dirt tasks that are detected and assigned as goals to the participating robots. Is in this node where the aggregated knowledge and self-adaptation features of our testbed come into play.

The goal manager keeps track of all generated dirt locations in the context, the detected dirt locations and the assigned goals to each robot. When a new dirt task is spawned, the dirt generator creates a dirt model in the simulation environment that poses collision properties, therefore the goal manager needs to maintain a list of all dirt locations and its associated model in order to be able to update and/or delete the model as needed. Whenever the goal manager receives a list of detected objects, it iterates over the dirt list and compares the pose of each element with the poses of the detected objects; if they match, the dirt model must be updated to disable its associated collision property, otherwise the robot taking care of this task will not be able to move into the dirt model’s location and "clean" the dirt up.

If knowledge aggregation is not enabled, as soon as a dirt tile is detected it is considered as a goal. Then, the goal manager updates the dirt model and publishes a notification to start the bidding process and set this task as a goal for the best bidder robot. On the other hand, if knowledge aggregation is enabled, the manager queries the Knowledge aggregator node, requesting information about the associated subjective opinion to the current dirt task (its location in the map) being considered. In this case, the goal manager will only start the bidding process if the belief mass distribution, uncertainty and expected probability values pass the given threshold limits. When a robot notifies that a goal has been attained, the goal manager deletes the dirt model from the simulation environment and updates the internal tracking lists accordingly.

## ROS Topics

### Publishes to:
* `/dirt_and_goals` [[goal_manager_msgs/GoalObjectList.msg](/ros/src/goal_manager_msgs/msg/GoalObjectList.msg))]: Publishes a list of the current tasks to be handled.
* `new_goal` [[commons_msgs/Goal.msg](/ros/src/commons_msgs/msg/Goal.msg))]: Publishes info about a new tasks to be handle by the best bidder robot.
* `/active_tasks` [[goal_manager_msgs/GoalObjectList.msg](/ros/src/goal_manager_msgs/msg/GoalObjectList.msg))]: Publishes a list of all the current tasks (regardless of categorized as a goal, undetected or detected dirt) as Goal types.

### Subscribed to:
* `/new_dirt` [[goal_manager_msgs/DirtModel.msg](/ros/src/goal_manager_msgs/msg/DirtModel.msg))]: Listen for info about new dirt created.
* `/new_dirt_goalObject` [[goal_manager_msgs/GoalObject.msg](/ros/src/goal_manager_msgs/msg/GoalObject.msg))]: Listens for new tasks that have been created and whose information is of goal object type.
* `/goal_attained` [[commons_msgs/Goal.msg](/ros/src/commons_msgs/msg/Goal.msg))]: Listen for the notification of a goal completed.
* `/detected_dirt` [[goal_manager_msgs/GoalObject.msg](/ros/src/goal_manager_msgs/msg/GoalObject.msg))]: Listen for info about dirt locations detected.
* `/new_dirt_goalObject` [[goal_manager_msgs/GoalObjectList.msg](/ros/src/goal_manager_msgs/msg/GoalObjectList.msg))]: The same content as in the new_dirt topic but of Goal Object type. This type includes further information such as about false positives.

## Launch parameters
The following parameters need to be provided to initialize the node.  
* `sl_threshold`: Expected probability threshold value to be used to handle new tasks 

These parameters can be provided/edited in the corresponding master launch file (see [test directory](/ros/test)).

## Known issue
When the Goal Manager node is launched, it creates an instance of the Knowledge Aggregator class. Both, the goal manager and knowledge aggregator make use of the occupancy map to initialize the testbed's context map and other variables. Until this occupancy map is published and read by this node, the corresponding variables are not initialized but accessed by other methods in the knowledge aggregator class when it receives observations from the participating robots. This raises the exception `AttributeError: 'KnowledgeAggregator' object has no attribute 'occupancy_map'`. Despite this exception, the node is kept alive, and the problem disappears as soon as the occupancy map info is received and the corresponding variables are initialized.

TODO: To solve this issue, we can either delay the start of the node or separate the knowledge aggregator and goal manager nodes.

# Knowledge Aggregator
Author: Sergio Quijano and Malte Neuss

## Description

The Knowledge Aggregator node is responsible for the aggregation of the partial observations coming from the participating robots to generate and store a new, common, global knowledge about the context.

When the system is first initialized, the run-time context model does not contain information that the Goal Manager can use to carry out its tasks. We model this situation by creating a vacuous subjective opinion for each cell of the context grid map. This initialization serves two purposes in our subjective logic approach: (1) a vacuous opinion will inform the Goal Manager that there is no previous knowledge about a context variable and (2) when the first knowledge aggregation is executed, the existing vacuous opinions do not influence in the final result.

The knowledge aggregation takes place each time a robot informs the Knowledge Aggregator of a made partial observation of the context. For the cells covered in the partial observation, subjective opinions are issued. Additionally, the Knowledge Aggregator extracts the previously-stored opinions for the corresponding cells from the Knowledge. For each cell, both opinions are then fused and aggregated, and the run-time context model in the Knowledge is updated accordingly. By default, this knowledge aggregation processes uses a combination of the CBF and CCF Subjective Logic Fusion operators, which is defined as follows: if the opinion type of the stored opinion and the new opinion differ and the uncertainty of the stored opinion is less then 0.1 the CCF operator is used. Otherwise the CBF operator is used. The opinion type of an opinion can either be occupied (if the belief mass is greater or equal to the disbelief mass) or unoccupied (if the disbelief mass is greater than the belief mass).

## ROS Topics

### Subscribed to:
* `/dirt_and_goals` [[goal_manager_msgs/GoalObjectList.msg](/ros/src/goal_manager_msgs/msg/GoalObjectList.msg))]: Listen for a list of the current tasks. Currently this list is not used, but we keep it as it may be useful to associate a partial observation to a taks's cell
* `/partial_observation` [[knowledge_aggregator_msgs/PartialObservation.msg](/ros/src/knowledge_aggregator_msgs/msg/PartialObservation.msg))]: Listen for a list of partial observations published by the robots

### Publishes to:
* `/opinions_map` [[knowledge_aggregator_msgs/SOList.msg](/ros/src/knowledge_aggregator_msgs/msg/SOList.msg))]: Publishes a list of all current opinions. Currently, this is only used for data collection and analysis.

## Launch parameters
The following parameters need to be provided to initialize the node.  
* `subjective_logic_operator`: Fusion operator to use for knowledge aggregation
* `sl_classpath`: Classpath indicating where the Subjective Logic library (JAR) is located

These parameters can be provided/edited in the corresponding master launch file (see [test directory](/ros/test)). 
