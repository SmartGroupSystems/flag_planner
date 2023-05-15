roslaunch px4ctrl run_ctrl.launch & sleep 2;
rosbag record -O planning_record /vins_fusion/imu_propagate /position_cmd /grid_map/occupancy_inflate /traj_vis /astar_node/grid_path


