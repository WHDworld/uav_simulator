#ifndef LIDAR_RENDER_CUH
#define LIDAR_RENDER_CUH

#include <cstdlib>
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <iostream>
#include <vector>

/**
 * @brief LiDAR render parameters for CUDA kernel
 */
struct LidarParameter {
  int point_number;           // Number of map points
  
  // Field of View
  float horizontal_fov;       // 360 degrees = 2*PI
  float vertical_fov_min;     // -7 degrees
  float vertical_fov_max;     // +52 degrees
  
  // Range
  float min_range;
  float max_range;
  
  // Output grid size for 360-degree coverage
  int horizontal_bins;        // Number of horizontal angle bins
  int vertical_bins;          // Number of vertical angle bins
  
  // Pose
  float r[3][3];              // Rotation matrix (lidar to world)
  float t[3];                 // Translation (lidar position in world)
  
  // Inflate radius for point rendering
  float radius;
};

/**
 * @brief CUDA-accelerated LiDAR point cloud renderer
 * 
 * Renders a 360-degree LiDAR scan by projecting map points into
 * spherical coordinates and finding the nearest point for each direction.
 */
class LidarRender {
public:
  LidarRender();
  ~LidarRender();

  /**
   * @brief Set LiDAR parameters
   */
  void set_para(float h_fov, float v_fov_min, float v_fov_max,
                float min_range, float max_range,
                int h_bins, int v_bins, float radius);

  /**
   * @brief Set map point cloud data (upload to GPU)
   */
  void set_data(std::vector<float>& cloud_data);

  /**
   * @brief Render LiDAR scan from given pose
   * @param transformation 4x4 transformation matrix (row-major)
   * @param host_range_ptr Output range values (h_bins * v_bins)
   */
  void render_pose(double* transformation, float* host_range_ptr);

  /**
   * @brief Get the total number of bins
   */
  int get_total_bins() const { return parameter.horizontal_bins * parameter.vertical_bins; }

private:
  int cloud_size;

  // Map data on device
  float3* host_cloud_ptr;
  float3* dev_cloud_ptr;
  bool has_devptr;

  // Parameters
  LidarParameter parameter;
  LidarParameter* parameter_devptr;
};

#endif // LIDAR_RENDER_CUH

