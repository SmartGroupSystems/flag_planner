roslaunch px4ctrl run_ctrl.launch & sleep 2;
rosbag record -O control_record /vins_fusion/imu_propagate /debugPx4ctrl

