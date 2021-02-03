#!/bin/sh

gnome-terminal --tab -- bash -c "roslaunch master_5r.launch gen_seed:=110 use_sl:=true sl_operator:=C
BF adaptive_scheduling:=true spawn_interval:=30; exec bash"

gnome-terminal --tab --working-directory=/home/malte/catkin_ws/src/knowledge_aggregation/ros/results/data -- bash -c "rosbag record -O r5_sl1_fp1_fn1_spi30_try /active_tasks /opinion_map /modified_occupancy_grid /robot_0/amcl_pose /robot_1/amcl_pose /robot_2/amcl_pose /robot_3/amcl_pose /robot_44/amcl_pose /all_completed_tasks; exec bash"



