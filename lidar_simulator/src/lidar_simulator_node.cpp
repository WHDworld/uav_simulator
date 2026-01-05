/**
 * @file lidar_simulator_node.cpp
 * @brief ROS node for CUDA-accelerated LiDAR simulation
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "lidar_simulator.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_simulator_node");
  ros::NodeHandle nh("~");
  
  ROS_INFO("[LidarSimulatorNode] Starting CUDA-accelerated LiDAR simulator...");
  
  // Get map file path
  std::string map_file;
  if (!nh.getParam("/map_config/map_file", map_file)) {
    ROS_ERROR("[LidarSimulatorNode] map_file parameter not set!");
    return -1;
  }
  
  // Handle relative vs absolute paths
  if (map_file[0] != '/') {
    // Check if it's a .pcd file, if not try to find .pcd version
    std::string map_render_path = ros::package::getPath("map_render");
    
    // Try original extension first
    std::string full_path = map_render_path + "/resource/" + map_file;
    
    // If .stl, try .pcd instead
    if (map_file.find(".stl") != std::string::npos) {
      std::string pcd_file = map_file.substr(0, map_file.find(".stl")) + ".pcd";
      std::string pcd_path = map_render_path + "/resource/" + pcd_file;
      
      // Check if .pcd exists
      std::ifstream f(pcd_path);
      if (f.good()) {
        map_file = pcd_path;
        ROS_INFO("[LidarSimulatorNode] Using .pcd version: %s", map_file.c_str());
      } else {
        ROS_WARN("[LidarSimulatorNode] No .pcd version found for %s, LiDAR will not work", 
                 full_path.c_str());
        return -1;
      }
    } else {
      map_file = full_path;
    }
  }
  
  ROS_INFO("[LidarSimulatorNode] Loading map from: %s", map_file.c_str());
  
  // Load map pointcloud
  pcl::PointCloud<pcl::PointXYZ> map_cloud;
  int status = -1;
  
  if (map_file.find(".pcd") != std::string::npos) {
    status = pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, map_cloud);
  } else if (map_file.find(".ply") != std::string::npos) {
    status = pcl::io::loadPLYFile<pcl::PointXYZ>(map_file, map_cloud);
  } else {
    ROS_ERROR("[LidarSimulatorNode] Unsupported map file format. Use .pcd or .ply");
    return -1;
  }
  
  if (status == -1) {
    ROS_ERROR("[LidarSimulatorNode] Failed to load map file: %s", map_file.c_str());
    return -1;
  }
  
  ROS_INFO("[LidarSimulatorNode] Loaded map with %lu points", map_cloud.size());
  
  // Initialize LiDAR simulator
  LidarSimulator::Ptr simulator(new LidarSimulator());
  if (simulator->initialize(nh) != 0) {
    ROS_ERROR("[LidarSimulatorNode] Failed to initialize LiDAR simulator");
    return -1;
  }
  
  simulator->setMapCloud(map_cloud, true);  // Add ground floor
  
  ROS_INFO("[LidarSimulatorNode] LiDAR simulator is running");
  
  ros::spin();
  
  return 0;
}

