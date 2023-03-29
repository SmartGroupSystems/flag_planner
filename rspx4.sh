sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch realsense2_camera rs_camera.launch & sleep 5;
roslaunch mavros px4.launch & sleep 5;
roslaunch vins fast_drone_250.launch
wait;
