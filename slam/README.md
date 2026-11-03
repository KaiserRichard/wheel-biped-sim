# Live RGB-D SLAM kit

This ROS 2 launch description connects a RealSense color and aligned-depth stream to RGB-D odometry and RTAB-Map. It expects ROS 2 Jazzy and the RealSense, RTAB-Map, and launch packages to be installed on the target computer.

Run `ros2 launch slam/live_slam.launch.py` from a ROS 2 package that installs this launch file. No robot, camera, or camera frames were available for validation; live performance and map quality are UNKNOWN. The launch kit is a starting point and will need adaptation to the camera mount and package versions.
