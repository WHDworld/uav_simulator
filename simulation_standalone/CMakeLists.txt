cmake_minimum_required(VERSION 3.0.2)
project(simulation_standalone)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  quadrotor_msgs
)

catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/waypoint_control.py
  scripts/depth_to_pointcloud.py
  scripts/rgb_camera_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files and configs
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

