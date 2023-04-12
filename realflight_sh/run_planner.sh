#! /usr/bin/env bash
set -e
RUN_AFTER_BASHRC="roslaunch mapping Mapping.launch" gnome-terminal --title="Local Mapping" --tab &
sleep 1 ;
RUN_AFTER_BASHRC="roslaunch grid_path_searcher Astar.launch" gnome-terminal --title="Astar" --tab & 
sleep 1 ;
RUN_AFTER_BASHRC="roslaunch bspline_race traj_testing.launch" gnome-terminal --title="Bspline" --tab &
sleep 1;
RUN_AFTER_BASHRC="rosrun bspline_race traj_server" gnome-terminal --title="Traj server" --tab &
sleep 1;
wait
exit 0
