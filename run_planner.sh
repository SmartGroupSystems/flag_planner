#! /usr/bin/env bash
set -e

RUN_AFTER_BASHRC="roslaunch so3_quadrotor_simulator simulator_example.launch" gnome-terminal --title="Simulator" --tab &
sleep 1 ;
RUN_AFTER_BASHRC="roslaunch mockamap post2d.launch" gnome-terminal --title="Simulator" --tab &
sleep 1;
RUN_AFTER_BASHRC="roslaunch mapping mapping.launch" gnome-terminal --title="Local Mapping" --tab &
sleep 1 ;
RUN_AFTER_BASHRC="roslaunch grid_path_searcher astar_node.launch" gnome-terminal --title="Astar" --tab & 
sleep 1 ;
RUN_AFTER_BASHRC="roslaunch bspline_race traj_testing.launch" gnome-terminal --title="Bspline" --tab &
sleep 1;
RUN_AFTER_BASHRC="rosrun so3_control control_bspline" gnome-terminal --title="Bspline" --tab &
sleep 1;
wait
exit 0
