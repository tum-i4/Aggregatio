#!/bin/bash

# the following parameters are not altered
use_sl=false
false_positive=true
false_negative=true
adaptive_scheduling=true
#robot_0_fpp=0.1
robot_0_fnp=0.2
sl_threshold=80

# Initial values of seed (incremented by 1)
gen_seed=71

# kill previous sessions if they exist.			
tmux kill-session -a -t saving
tmux kill-session -a -t ros


# Using Mutex so SSH compatible
num=1
for i in `seq 5`;
do
	# Initial value of spawn_interval (in seconds) (inremented by 10s)
	spawn_interval=60
	robot_0_fpp=0.1
	for j in `seq 9`;
	do
		# Bash statement
		test_str="Test Num. "$num": gen_seed = "$gen_seed" spawn_interval = "$spawn_interval
		echo $test_str

		filename="r1_sl"$use_sl"_fp"$false_positive"_fn"$false_negative"_spi"$spawn_interval"_seed"$gen_seed"_r0fpp"$robot_0_fpp"_r0fnp"$robot_0_fnp"_thresh"$sl_threshold
		FILE="../results/data/"$filename".bag"
		echo $FILE
		
		# check if file already exists, then ext loop iteration
		if [ -f "$FILE" ];
		then
			echo "File already exists, move on to next test"
		else
			# Run test in detached mutex session:	
			tmux new-session -d -s "ros" "roslaunch master.launch gen_seed:=$gen_seed use_sl:=$use_sl false_positive:=$false_positive false_negative:=$false_negative adaptive_scheduling:=$adaptive_scheduling spawn_interval:=$spawn_interval robot_0_fpp:=$robot_0_fpp robot_0_fnp:=$robot_0_fnp sl_threshold:=$sl_threshold"

			# Storing data in detached mutex session (first go to correct directory)
			tmux new-session -d -s "saving" "cd /home/ge73xus/catkin_ws/src/knowledge_aggregation/ros/results/data; rosbag record -O "$filename" /active_tasks /opinion_map /modified_occupancy_grid /robot_0/amcl_pose /partial_observation /all_completed_tasks"

			# Let the ROS test run for 45 minuites before stopping it
			sleep 45m
			rosnode kill -a; killall -9 rosmaster; killall -9 roscore

			# wait 20 seconds to make sure that all the rosnodes have stopped properly
			sleep 20s

			# delete the tmux sessions (should automatically close when killing ros)
			tmux kill-session -a -t saving
			tmux kill-session -a -t ros
		fi
		
		# increment variables
		let num=`expr $num + 1`
		#let spawn_interval=$((spawn_interval + 45))
		robot_0_fpp=$(echo $robot_0_fpp + 0.1 |bc -l | sed -e 's/^\./0./' -e 's/^-\./-0./');
	done

	# increment variable
	gen_seed=$((gen_seed + 1))
done




