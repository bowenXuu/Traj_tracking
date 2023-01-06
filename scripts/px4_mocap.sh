# sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 & sleep 5;
roslaunch Traj_tracking mocap_K216.launch & sleep 2;
# roslaunch Traj_tracking rviz.launch;
wait;
