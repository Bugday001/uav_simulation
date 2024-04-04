# UAV-SIMULATOR
Simulate UAV with rviz.

## Features
- ROS2 humble support.
- Simulate UAV with rviz.
- A simple trajectory generator.
- 2D and 3D mode.
    - 2D mode: UAV can only move in z-y plane with PD control.
    - 3D mode: UAV can move in x-y-z plane with SE3 control.

## RUN
`ros2 launch uav_simulation sim_uav.launch.py`


## Acknowledgement
Thanks [rotors_simulator](https://github.com/ethz-asl/rotors_simulator), 
Minimum Snap Trajectory Generation and Control for Quadrotors,
Geometric Tracking Control of a Quadroror UAV on SE(3)
