roslaunch so3_quadrotor_simulator sim_test_control.launch & sleep 2;
roslaunch mavros px4.launch & sleep 2;
roslaunch px4ctrl run_ctrl_sim.launch & sleep 1;
rosbag record -O real_record /odom_visualization/pose /debugPx4ctrl

