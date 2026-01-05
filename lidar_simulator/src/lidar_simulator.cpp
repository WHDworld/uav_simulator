#include "lidar_simulator.h"
#include <cmath>
#include <algorithm>

int LidarSimulator::initialize(ros::NodeHandle& nh) {
  nh_ = nh;
  
  // Read LiDAR parameters from config
  nh_.param("/uav_model/sensing_parameters/lidar/horizontal_fov", horizontal_fov_, 2.0 * M_PI);
  nh_.param("/uav_model/sensing_parameters/lidar/vertical_fov_min", vertical_fov_min_, -7.0 * M_PI / 180.0);
  nh_.param("/uav_model/sensing_parameters/lidar/vertical_fov_max", vertical_fov_max_, 52.0 * M_PI / 180.0);
  nh_.param("/uav_model/sensing_parameters/lidar/min_range", min_range_, 0.1);
  nh_.param("/uav_model/sensing_parameters/lidar/max_range", max_range_, 30.0);
  nh_.param("/uav_model/sensing_parameters/lidar/target_points", target_points_, 10000);
  nh_.param("/uav_model/sensing_parameters/lidar/render_rate", render_rate_, 10.0);
  nh_.param("/uav_model/sensing_parameters/lidar/range_noise_std", range_noise_std_, 0.02);
  nh_.param("/uav_model/sensing_parameters/lidar/add_noise", add_noise_, true);
  
  // Render parameters
  nh_.param("/lidar_simulator/pose_rate", pose_rate_, 30.0);
  nh_.param("/lidar_simulator/inflate_radius", inflate_radius_, 0.1);
  nh_.param("/lidar_simulator/verbose", verbose_, false);
  
  // Frame settings
  nh_.param("/lidar_simulator/world_frame", world_frame_, std::string("world"));
  nh_.param("/lidar_simulator/lidar_frame", lidar_frame_, std::string("lidar"));
  
  // Calculate grid resolution based on target points and FOV
  // Approximate: target_points = h_bins * v_bins, with aspect ratio based on FOV
  double v_fov = vertical_fov_max_ - vertical_fov_min_;
  double aspect = horizontal_fov_ / v_fov;
  vertical_bins_ = (int)std::sqrt(target_points_ * 2.0 / aspect);  // More bins to allow downsampling
  horizontal_bins_ = (int)(vertical_bins_ * aspect);
  
  // Ensure minimum bins
  horizontal_bins_ = std::max(horizontal_bins_, 360);
  vertical_bins_ = std::max(vertical_bins_, 60);
  
  total_bins_ = horizontal_bins_ * vertical_bins_;
  range_buffer_ = new float[total_bins_];
  
  // LiDAR to body transform (default: identity)
  body2lidar_ = Eigen::Matrix4d::Identity();
  std::vector<double> lidar_pose;
  if (nh_.getParam("/uav_model/sensing_parameters/lidar/lidar_to_body", lidar_pose) && lidar_pose.size() == 6) {
    Eigen::Vector3d trans(lidar_pose[0], lidar_pose[1], lidar_pose[2]);
    Eigen::AngleAxisd rollAngle(lidar_pose[3], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(lidar_pose[4], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(lidar_pose[5], Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rot = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
    body2lidar_.block<3, 3>(0, 0) = rot;
    body2lidar_.block<3, 1>(0, 3) = trans;
  }
  
  // Initialize noise
  if (add_noise_) {
    noise_dist_ = std::normal_distribution<double>(0.0, range_noise_std_);
    rng_.seed(std::random_device{}());
  }
  
  // Initialize CUDA renderer
  lidar_render_.set_para(horizontal_fov_, vertical_fov_min_, vertical_fov_max_,
                         min_range_, max_range_, horizontal_bins_, vertical_bins_, 
                         inflate_radius_);
  
  // Initialize state
  pose_initialized_ = false;
  map_initialized_ = false;
  lidar2world_ = Eigen::Matrix4d::Identity();
  
  // Set up subscribers and publishers
  odom_sub_ = nh_.subscribe("/uav_simulator/odometry", 10, 
                            &LidarSimulator::odometryCallback, this);
  
  lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 10);
  lidar_pose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/lidar/pose", 10);
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar/map", 1, true);
  
  // Set up timers
  render_timer_ = nh_.createTimer(ros::Duration(1.0 / render_rate_), 
                                  &LidarSimulator::renderCallback, this);
  pose_timer_ = nh_.createTimer(ros::Duration(1.0 / pose_rate_),
                                &LidarSimulator::pubLidarPose, this);
  
  ROS_INFO("[LidarSimulator] Initialized with parameters:");
  ROS_INFO("  Horizontal FOV: %.1f deg", horizontal_fov_ * 180.0 / M_PI);
  ROS_INFO("  Vertical FOV: [%.1f, %.1f] deg", 
           vertical_fov_min_ * 180.0 / M_PI, vertical_fov_max_ * 180.0 / M_PI);
  ROS_INFO("  Range: [%.2f, %.2f] m", min_range_, max_range_);
  ROS_INFO("  Target points: %d, Grid: %dx%d", target_points_, horizontal_bins_, vertical_bins_);
  ROS_INFO("  Render rate: %.1f Hz", render_rate_);
  
  return 0;
}

void LidarSimulator::setMapCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, bool add_floor) {
  if (cloud.points.empty()) {
    ROS_WARN("[LidarSimulator] Empty point cloud provided");
    return;
  }
  
  cloud_data_.clear();
  for (const auto& pt : cloud.points) {
    cloud_data_.push_back(pt.x);
    cloud_data_.push_back(pt.y);
    cloud_data_.push_back(pt.z);
  }
  
  // Optionally add ground floor
  if (add_floor) {
    Eigen::Vector2d mmin(0, 0), mmax(0, 0);
    for (const auto& pt : cloud) {
      mmin[0] = std::min(mmin[0], (double)pt.x);
      mmin[1] = std::min(mmin[1], (double)pt.y);
      mmax[0] = std::max(mmax[0], (double)pt.x);
      mmax[1] = std::max(mmax[1], (double)pt.y);
    }
    
    for (double x = mmin[0]; x < mmax[0]; x += 0.5) {
      for (double y = mmin[1]; y < mmax[1]; y += 0.5) {
        cloud_data_.push_back(x);
        cloud_data_.push_back(y);
        cloud_data_.push_back(0.0);
      }
    }
  }
  
  // Upload to GPU
  lidar_render_.set_data(cloud_data_);
  map_initialized_ = true;
  
  // Publish map for visualization
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(cloud, map_msg);
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = world_frame_;
  map_pub_.publish(map_msg);
  
  ROS_INFO("[LidarSimulator] Map cloud set with %lu points", cloud.points.size());
}

void LidarSimulator::odometryCallback(const nav_msgs::Odometry& odom) {
  last_odom_ = odom;
  last_odom_stamp_ = odom.header.stamp;
  
  // Build body to world transform
  Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(odom.pose.pose.orientation.w,
                       odom.pose.pose.orientation.x,
                       odom.pose.pose.orientation.y,
                       odom.pose.pose.orientation.z);
  body2world.block<3, 3>(0, 0) = q.toRotationMatrix();
  body2world(0, 3) = odom.pose.pose.position.x;
  body2world(1, 3) = odom.pose.pose.position.y;
  body2world(2, 3) = odom.pose.pose.position.z;
  
  // Compute LiDAR to world transform
  lidar2world_ = body2world * body2lidar_;
  lidar2world_quat_ = Eigen::Quaterniond(lidar2world_.block<3, 3>(0, 0));
  
  pose_initialized_ = true;
  
  if (verbose_) {
    ROS_INFO_ONCE("[LidarSimulator] Received first odometry");
  }
}

void LidarSimulator::pubLidarPose(const ros::TimerEvent& event) {
  if (!pose_initialized_)
    return;
  
  geometry_msgs::TransformStamped pose_msg;
  pose_msg.header.stamp = last_odom_stamp_;
  pose_msg.header.frame_id = world_frame_;
  pose_msg.child_frame_id = lidar_frame_;
  pose_msg.transform.translation.x = lidar2world_(0, 3);
  pose_msg.transform.translation.y = lidar2world_(1, 3);
  pose_msg.transform.translation.z = lidar2world_(2, 3);
  pose_msg.transform.rotation.w = lidar2world_quat_.w();
  pose_msg.transform.rotation.x = lidar2world_quat_.x();
  pose_msg.transform.rotation.y = lidar2world_quat_.y();
  pose_msg.transform.rotation.z = lidar2world_quat_.z();
  
  lidar_pose_pub_.publish(pose_msg);
}

void LidarSimulator::renderCallback(const ros::TimerEvent& event) {
  if (!(pose_initialized_ && map_initialized_))
    return;
  
  renderLidarScan();
}

void LidarSimulator::renderLidarScan() {
  double start_time = ros::Time::now().toSec();
  
  // Prepare transformation matrix (row-major for CUDA)
  double transform[16];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      transform[i * 4 + j] = lidar2world_(i, j);
    }
  }
  
  // Render using CUDA
  lidar_render_.render_pose(transform, range_buffer_);
  
  // Convert range buffer to point cloud
  pcl::PointCloud<pcl::PointXYZI> scan_cloud;
  scan_cloud.reserve(total_bins_);
  
  double h_step = horizontal_fov_ / horizontal_bins_;
  double v_step = (vertical_fov_max_ - vertical_fov_min_) / vertical_bins_;
  
  for (int v = 0; v < vertical_bins_; v++) {
    double v_angle = vertical_fov_min_ + (v + 0.5) * v_step;
    
    for (int h = 0; h < horizontal_bins_; h++) {
      int idx = v * horizontal_bins_ + h;
      float range = range_buffer_[idx];
      
      // Skip invalid ranges
      if (range > max_range_ || range < min_range_)
        continue;
      
      // Add noise if enabled
      if (add_noise_) {
        range += noise_dist_(rng_);
        range = std::max((float)min_range_, range);
      }
      
      // Convert from spherical to Cartesian (LiDAR frame)
      double h_angle = -M_PI + (h + 0.5) * h_step;
      double cos_v = std::cos(v_angle);
      double sin_v = std::sin(v_angle);
      double cos_h = std::cos(h_angle);
      double sin_h = std::sin(h_angle);
      
      // LiDAR frame: x-forward, y-left, z-up
      Eigen::Vector4d pt_lidar;
      pt_lidar(0) = range * cos_v * cos_h;
      pt_lidar(1) = range * cos_v * sin_h;
      pt_lidar(2) = range * sin_v;
      pt_lidar(3) = 1.0;
      
      // Transform to world frame
      Eigen::Vector4d pt_world = lidar2world_ * pt_lidar;
      
      pcl::PointXYZI pt;
      pt.x = pt_world(0);
      pt.y = pt_world(1);
      pt.z = pt_world(2);
      pt.intensity = 100.0f * (1.0f - range / max_range_);
      
      scan_cloud.push_back(pt);
    }
  }
  
  // Downsample to target point count
  if ((int)scan_cloud.size() > target_points_) {
    downsamplePointCloud(scan_cloud, target_points_);
  }
  
  // Publish
  sensor_msgs::PointCloud2 scan_msg;
  pcl::toROSMsg(scan_cloud, scan_msg);
  scan_msg.header.stamp = last_odom_stamp_;
  scan_msg.header.frame_id = world_frame_;
  lidar_pub_.publish(scan_msg);
  
  if (verbose_) {
    double elapsed = (ros::Time::now().toSec() - start_time) * 1000.0;
    ROS_INFO("[LidarSimulator] Rendered %lu points in %.2f ms", scan_cloud.size(), elapsed);
  }
}

void LidarSimulator::downsamplePointCloud(pcl::PointCloud<pcl::PointXYZI>& cloud, int target_points) {
  if ((int)cloud.size() <= target_points)
    return;
  
  // Random shuffle and take first N points
  std::vector<int> indices(cloud.size());
  for (size_t i = 0; i < cloud.size(); i++) {
    indices[i] = i;
  }
  
  std::shuffle(indices.begin(), indices.end(), rng_);
  
  pcl::PointCloud<pcl::PointXYZI> downsampled;
  downsampled.reserve(target_points);
  
  for (int i = 0; i < target_points; i++) {
    downsampled.push_back(cloud.points[indices[i]]);
  }
  
  cloud = downsampled;
}

