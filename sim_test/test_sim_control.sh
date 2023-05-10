roslaunch so3_quadrotor_simulator simulator_example.launch & sleep 1;
rosrun so3_control control_example &sleep 1;
rosbag record -O sim_record /odom_visualization/pose /so3_cmd

