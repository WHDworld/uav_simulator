#ifndef LIDAR_SIMULATOR_H
#define LIDAR_SIMULATOR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <random>
#include <memory>
#include <vector>

#include "lidar_render.cuh"

/**
 * @brief CUDA-accelerated LiDAR simulator
 * 
 * Simulates a 360-degree LiDAR (like Livox Mid-360) using GPU-accelerated
 * point cloud projection. Generates a spherical scan and randomly downsamples
 * to target point count while maintaining uniform angular coverage.
 */
class LidarSimulator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef std::shared_ptr<LidarSimulator> Ptr;
  
  LidarSimulator() = default;
  ~LidarSimulator() = default;
  
  /**
   * @brief Initialize the LiDAR simulator
   * @param nh ROS node handle
   * @return 0 on success, -1 on failure
   */
  int initialize(ros::NodeHandle& nh);
  
  /**
   * @brief Set the map pointcloud for rendering
   * @param cloud Map pointcloud
   * @param add_floor Whether to add a ground plane
   */
  void setMapCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, bool add_floor = false);

private:
  // Callbacks
  void odometryCallback(const nav_msgs::Odometry& odom);
  void renderCallback(const ros::TimerEvent& event);
  void pubLidarPose(const ros::TimerEvent& event);
  
  /**
   * @brief Render LiDAR point cloud
   */
  void renderLidarScan();
  
  /**
   * @brief Random downsample point cloud to target size
   * Maintains uniform angular distribution
   */
  void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZI>& cloud, int target_points);
  
  // ROS components
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher lidar_pub_;
  ros::Publisher lidar_pose_pub_;
  ros::Publisher map_pub_;
  ros::Timer render_timer_;
  ros::Timer pose_timer_;
  
  // CUDA LiDAR renderer
  LidarRender lidar_render_;
  float* range_buffer_;
  int total_bins_;
  
  // Map data
  std::vector<float> cloud_data_;
  bool map_initialized_;
  
  // State
  bool pose_initialized_;
  Eigen::Matrix4d lidar2world_;
  Eigen::Matrix4d body2lidar_;
  Eigen::Quaterniond lidar2world_quat_;
  ros::Time last_odom_stamp_;
  nav_msgs::Odometry last_odom_;
  
  // LiDAR parameters
  double horizontal_fov_;
  double vertical_fov_min_;
  double vertical_fov_max_;
  double min_range_;
  double max_range_;
  int target_points_;
  double render_rate_;
  double pose_rate_;
  
  // Grid parameters (for CUDA rendering)
  int horizontal_bins_;
  int vertical_bins_;
  double inflate_radius_;
  
  // Noise
  bool add_noise_;
  double range_noise_std_;
  std::default_random_engine rng_;
  std::normal_distribution<double> noise_dist_;
  
  // Frame IDs
  std::string world_frame_;
  std::string lidar_frame_;
  
  // Verbose
  bool verbose_;
};

#endif // LIDAR_SIMULATOR_H

