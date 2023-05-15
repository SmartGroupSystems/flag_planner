rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
#! /usr/bin/env bash
set -e
RUN_AFTER_BASHRC="roslaunch mapping Mapping.launch" gnome-terminal --title="Local Mapping" --tab &
sleep 1 ;
RUN_AFTER_BASHRC="roslaunch grid_path_searcher Astar.launch" gnome-terminal --title="Astar" --tab & 
sleep 1 ;
RUN_AFTER_BASHRC="roslaunch bspline_race traj_testing.launch" gnome-terminal --title="Bspline" --tab &
sleep 1;
RUN_AFTER_BASHRC="rosrun bspline_race traj_server" gnome-terminal --title="Traj server" --tab &
sleep 2;
rosbag record -O planning_record /vins_fusion/imu_propagate /position_cmd /grid_map/occupancy_inflate /traj_vis /astar_node/grid_path
wait
exit 0
