roslaunch px4ctrl run_ctrl.launch & sleep 2;
rosbag record -O setpoint_vins /vins_fusion/imu_propagate /debugPx4ctrl

