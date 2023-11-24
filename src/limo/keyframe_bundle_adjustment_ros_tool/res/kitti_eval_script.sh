#!/bin/bash

# Paths of the destination paths, KITTI_EVAL PARAM_SET is used for parameter tuning.
# If unset it will just evaluate to ""
dst_prepath="/home/graeter/kitti_eval_tune_params/kitti_results_mono_lidar$1_$2_$3/"
# mkdir if not available
mkdir -p $dst_prepath

# dump params
echo "depth_loss: $1\nreprojection_loss: $2\nshrubbery_weight: $3" > "$dst_prepath/params.yaml"

echo ""
echo ""
echo "Eval All KITTI Sequences"

# Rosbags
bagpath="/media/graeter/data/odometry/kitti_bags/all_with_labels/"

bagindex[0]="00"
bagindex[1]="01"
bagindex[2]="02"
bagindex[3]="03"
bagindex[4]="04"
bagindex[5]="05"
bagindex[6]="06"
bagindex[7]="07"
bagindex[8]="08"
bagindex[9]="09"
bagindex[10]="10"
bagindex[11]="11"
bagindex[12]="12"
bagindex[13]="13"
bagindex[14]="14"
bagindex[15]="15"
bagindex[16]="16"
bagindex[17]="17"
bagindex[18]="18"
bagindex[19]="19"
bagindex[20]="20"
bagindex[21]="21"

# _____ Run evaluation for all bags _____

# source path for main library.
source $HOME/workspaces/keyframe_ba/devel/setup.bash
# newer version of rosbag play supports rate-control-topics, so build it yourself and source it.
source $HOME/workspaces/rosbag_play/devel/setup.bash --extend

tmux kill-server

# core
tmux new-session -d -s core "roscore"
echo "Started roscore"

for bag in ${bagindex[@]}
do
	echo "Start eval sequ $bag"
	# _____ Run main processes _____ 
	# export variable for tmux
	export bag
	sleep 2
	# start the odometry (rosnode) and close the newly created window after the process has been killed:
	tmux new-session -d -s my_main "roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch | tee /tmp/log_$bag.txt"
	#tmux new-session -d -s my_main "roslaunch demo_keyframe_bundle_adjustment_meta kitti_mono_standalone.launch | tee /tmp/log_$bag.txt"
	sleep 4
	
	# reconfigure dump path by dynamic reconfigure
	dump_path_name=$dst_prepath$bag".txt"
	export dump_path_name
	tmux new-session -d -s dyn_set0 "rosrun dynamic_reconfigure dynparam set /mono_lidar dump_path $dump_path_name;sleep 2"
	
	if ! [ -z "$1" ]
	then
		echo "set depth_thres to $1"
		export $1
		tmux new-session -d -s dyn_set1 "rosrun dynamic_reconfigure dynparam set /mono_lidar robust_loss_depth_thres $1;sleep 1"
	fi
	if ! [ -z "$2" ]
	then
		echo "set reprojection_thres to $2"
		export $2
		tmux new-session -d -s dyn_set2 "rosrun dynamic_reconfigure dynparam set /mono_lidar robust_loss_reprojection_thres $2;sleep 1"
	fi
	if ! [ -z "$3" ]
	then
		echo "set shrubbery_weight to $3"
		export $3
		tmux new-session -d -s dyn_set1 "rosrun dynamic_reconfigure dynparam set /mono_lidar shrubbery_weight $3;sleep 1"
	fi

	# dump tf stuff so that we can get all tf frames that we want afterwards
	bag_path_name=$dst_prepath$bag"_tf.bag"
	export bag_path_name
	#tmux new-session -d -s record "rosbag record /tf /tf_static /clock /groundtruth_pose/pose /estimate/landmarks /estimate_prior/trajectory /estimate/complete_path /estimate/active_path /gt/trajectory -o $bag_path_name __name:=my_record"

	# starte rosbag
	bagpath_full=$bagpath$bag".bag"
	echo "Start bag: $bagpath_full"

	# better run rhe rosbag sowly/safe to avoid the risk of frame skip (kitti evaluation failes)
	rosrun rosbag play "$bagpath_full" --clock -r 0.2 -d 6 --rate-control-topic "/estimate/trajectory" --rate-control-max-delay 0.8
	#rosbag play "$bagpath_full" -r 0.1 -d 6 --clock

	# Kill monolidar gracefully to write all
	rosnode kill /mono_lidar
	# Kill rosbag record via ros. Only killing window will leave it in active state.	
	rosnode kill /my_record
	sleep 2

	# Move groudnplane record to output path
	mv /tmp/gp.txt $dst_prepath$bag"_gp.txt"

	tmux kill-session -t dyn_set0
	tmux kill-session -t dyn_set1
	tmux kill-session -t dyn_set2
	tmux kill-session -t record
	tmux kill-session -t my_main

	sleep 5
	
	echo "done with sequence $bag"
done

# Kill core
tmux kill-session -t core

# call kitti evaluation node
# /home/wilczynski/U/workspace/test2/devel/lib/kitti_devkit_tool/evaluate_odometry_all $dst_path_kitti_main
