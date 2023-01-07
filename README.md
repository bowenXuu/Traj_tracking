# Traj_tracking

# Observe like a Swan: A Bionic Aerial Neck-eye System for Active Visual-inertial State Estimation

## 1. Introduction
**Traj_tracking** as the basis for the implementation of the physical experiments in the above paper, is a good set of tools for MAVs to track a specific pre-defined trajectory.
- For MAVs developed using [px4](https://docs.px4.io/main/en/) as an autopilot.
- MAV flight control development using motion capture systems such as [Vicon](https://www.vicon.com/software/tracker/) or [Optitrack](https://optitrack.com/software/motive/) for positional feedback.
- Running requires trajectory message publication outside of this package, of type `trajectory_msgs/MultiDOFJointTrajectory`
.

Authors: To be added

If you use **Traj_tracking** for your academic research, please cite the following paper.
```
@ARTICLE{,  
  author={},  
  journal={},  
  title={},   
  year={},  
  volume={},  
  number={},  
  pages={},  
  doi={}
```



## 2. Installation
Tested on Ubuntu 16.04 and 18.04.

```
cd ~/catkin_ws/src
git clone https://github.com/bowenXuu/Traj_tracking.git
cd ..
catkin_make
```

## 3. Try it on your own MAV

### 3.1. Launch trajectory tracking
Before performing the following steps, make sure that:
- The motion capture is operating normally and the MAV is continuously receiving information about its own position.
- Pixhawk and on-board computer are properly connected
```bash
# Start the mavros node, connect the pixhawk to the nuc, and start receiving motion capture poses.
sh ./src/Traj_tracking/scripts/px4_mocap.sh
# Visualize poses reception and real-time setpoint
roslaunch Traj_tracking rviz.launch
```

### 3.2. RC function introduction

- Channel 5：Control Mode
  - Low：Entering STABLIZED
  - Medium：Start streaming setpoints
- Channel 6：Off-board mode
  - Low：MAV on lock
  - Medium：
    - OFFBOARD mode: Terminate all tasks and land in the same place
    - Other modes: No operation
  - High：Unlock and switch to OFFBOARD
- Channel 7：Emergency Mode
  - Low：No operation
  - High：Emergency stop

### 3.3. Select the function you want to use
**Function 1 (Used to test whether the position control of MAV is normal)**

Take off from any location and send a setpoint to control the movement of the MAV in real time by remote control after taking off
```bash
rosrun Traj_tracking rc_ctl
```
***Real flight control process：***
1. Make sure all channels 567 are switch to low, throttle adjusted to the lowest, start `rc_ctl` node
2. Channel 5 switch to in the send takeoff setpoint, at this time rviz should be visible on the setpoint in the aircraft directly above the 0.8 m height, at this time the throttle adjustment to the middle position
3. Channel 6 switch to high to unlock the aircraft, wait for the aircraft to take off, after successful takeoff will hover, before the subsequent operation
4. You can change the xyz position of the aircraft setpoint by controlling 123 channels
5. When the mission is over, Channel 6 switch to land.
6. After landing smoothly, execute Channel 7 switch to high, Channel 6 switch to low, Channel 5 switch to low, Channel 7 switch to low in turn to achieve reset.
---
**Function 2**

Take off from any location and control the MAV movement according to the pre-recorded trajectory information after take-off
```bash
rosrun Traj_tracking traj_ctl
```
***Real flight control process：***
1. Make sure all channels 567 are switch to low, throttle to the minimum, start `traj_ctl` node
2. Channel 5 switch to send the takeoff setpoint, at this time the setpoint should be visible on the rviz at 0.8m height directly above the aircraft
3. Channel 6 switch to high to unlock the aircraft, wait for the aircraft to take off, after successful takeoff, it will automatically fly to the first point of the preset trajectory
4. After reaching the starting point of the trajectory, the terminal will send ROS INFO: You can now send the trajectory message!
5. After seeing the prompt from the terminal, play the rosbag with the trajectory message, and the aircraft will follow the pre-planned route and time stamp for trajectory tracking
6. When the mission is over (the aircraft flies back to the takeoff point in a circle along the trajectory), channel 6 will switch to land.
7. After the landing is confirmed to be smooth, execute channel 7 switch to high, channel 6 switch to low, channel 5 switch to low and channel 7 switch to low in turn to achieve reset.

## 4. Notes
Find more things to keep in mind when flying here : **[flight instructions](./doc/NOTE.md)**.