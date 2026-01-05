#include "lidar_render.cuh"
#include <cmath>

/**
 * @brief CUDA kernel to initialize range image with max values
 */
__global__ void lidar_range_init(float* range_devptr, int total_bins) {
  const int idx = threadIdx.x + blockIdx.x * blockDim.x;
  if (idx >= total_bins)
    return;
  range_devptr[idx] = 999999.0f;  // Initialize with large value
}

/**
 * @brief CUDA kernel to project map points to LiDAR range image
 * 
 * For each map point, compute its direction from LiDAR origin,
 * convert to spherical coordinates, and update range image if closer.
 */
__global__ void lidar_render(float3* data_devptr, LidarParameter* para_devptr, 
                              float* range_devptr) {
  const int index = threadIdx.x + blockIdx.x * blockDim.x;
  const LidarParameter para = *para_devptr;
  
  if (index >= para.point_number)
    return;
  
  float3 world_point = data_devptr[index];
  
  // Transform point from world to LiDAR frame
  // First translate, then rotate by inverse
  float3 rel_point;
  rel_point.x = world_point.x - para.t[0];
  rel_point.y = world_point.y - para.t[1];
  rel_point.z = world_point.z - para.t[2];
  
  // Apply inverse rotation (transpose of rotation matrix)
  float3 lidar_point;
  lidar_point.x = rel_point.x * para.r[0][0] + rel_point.y * para.r[1][0] + rel_point.z * para.r[2][0];
  lidar_point.y = rel_point.x * para.r[0][1] + rel_point.y * para.r[1][1] + rel_point.z * para.r[2][1];
  lidar_point.z = rel_point.x * para.r[0][2] + rel_point.y * para.r[1][2] + rel_point.z * para.r[2][2];
  
  // Compute range
  float range = sqrtf(lidar_point.x * lidar_point.x + 
                      lidar_point.y * lidar_point.y + 
                      lidar_point.z * lidar_point.z);
  
  // Check range bounds
  if (range < para.min_range || range > para.max_range)
    return;
  
  // Compute horizontal angle (azimuth): atan2(y, x), range [-PI, PI]
  float h_angle = atan2f(lidar_point.y, lidar_point.x);
  
  // Compute vertical angle (elevation): asin(z / range), range [-PI/2, PI/2]
  float v_angle = asinf(lidar_point.z / range);
  
  // Check vertical FOV
  if (v_angle < para.vertical_fov_min || v_angle > para.vertical_fov_max)
    return;
  
  // Convert angles to bin indices
  // Horizontal: [-PI, PI] -> [0, horizontal_bins-1]
  float h_normalized = (h_angle + 3.14159265f) / para.horizontal_fov;
  int h_bin = (int)(h_normalized * para.horizontal_bins);
  h_bin = max(0, min(para.horizontal_bins - 1, h_bin));
  
  // Vertical: [v_fov_min, v_fov_max] -> [0, vertical_bins-1]
  float v_range = para.vertical_fov_max - para.vertical_fov_min;
  float v_normalized = (v_angle - para.vertical_fov_min) / v_range;
  int v_bin = (int)(v_normalized * para.vertical_bins);
  v_bin = max(0, min(para.vertical_bins - 1, v_bin));
  
  // Compute linear index
  int range_idx = v_bin * para.horizontal_bins + h_bin;
  
  // Atomic min to handle multiple points in same bin
  // Use int atomicMin on the float value (reinterpreted as int)
  // This works because IEEE 754 floats preserve ordering when reinterpreted as ints
  // for positive values
  int range_int = __float_as_int(range);
  atomicMin((int*)&range_devptr[range_idx], range_int);
  
  // Also update neighboring bins for point inflation
  float r_bins = para.radius / range * para.horizontal_bins / (2.0f * 3.14159265f);
  int r = (int)(r_bins + 0.5f);
  if (r > 0 && r <= 3) {
    for (int di = -r; di <= r; di++) {
      for (int dj = -r; dj <= r; dj++) {
        if (di == 0 && dj == 0) continue;
        
        int new_h = h_bin + dj;
        int new_v = v_bin + di;
        
        // Wrap horizontal (360 degrees)
        new_h = (new_h + para.horizontal_bins) % para.horizontal_bins;
        
        // Clamp vertical
        if (new_v < 0 || new_v >= para.vertical_bins) continue;
        
        int new_idx = new_v * para.horizontal_bins + new_h;
        atomicMin((int*)&range_devptr[new_idx], range_int);
      }
    }
  }
}

LidarRender::LidarRender()
    : cloud_size(0), host_cloud_ptr(nullptr), dev_cloud_ptr(nullptr), has_devptr(false) {
}

LidarRender::~LidarRender() {
  if (has_devptr) {
    free(host_cloud_ptr);
    cudaFree(dev_cloud_ptr);
    cudaFree(parameter_devptr);
  }
}

void LidarRender::set_para(float h_fov, float v_fov_min, float v_fov_max,
                           float min_range, float max_range,
                           int h_bins, int v_bins, float radius) {
  parameter.horizontal_fov = h_fov;
  parameter.vertical_fov_min = v_fov_min;
  parameter.vertical_fov_max = v_fov_max;
  parameter.min_range = min_range;
  parameter.max_range = max_range;
  parameter.horizontal_bins = h_bins;
  parameter.vertical_bins = v_bins;
  parameter.radius = radius;
}

void LidarRender::set_data(std::vector<float>& cloud_data) {
  cloud_size = cloud_data.size() / 3;
  parameter.point_number = cloud_size;
  
  host_cloud_ptr = (float3*)malloc(cloud_size * sizeof(float3));
  for (int i = 0; i < cloud_size; i++) {
    host_cloud_ptr[i] = make_float3(cloud_data[3 * i], 
                                     cloud_data[3 * i + 1], 
                                     cloud_data[3 * i + 2]);
  }
  
  cudaError err = cudaMalloc(&dev_cloud_ptr, cloud_size * sizeof(float3));
  if (err != cudaSuccess) {
    printf("CUDA Error: Failed to allocate device memory for point cloud\n");
    return;
  }
  
  err = cudaMemcpy(dev_cloud_ptr, host_cloud_ptr, cloud_size * sizeof(float3),
                   cudaMemcpyHostToDevice);
  if (err != cudaSuccess) {
    printf("CUDA Error: Failed to copy point cloud to device\n");
    return;
  }
  
  err = cudaMalloc(&parameter_devptr, sizeof(LidarParameter));
  if (err != cudaSuccess) {
    printf("CUDA Error: Failed to allocate device memory for parameters\n");
    return;
  }
  
  has_devptr = true;
}

void LidarRender::render_pose(double* transformation, float* host_range_ptr) {
  // Extract rotation and translation from transformation matrix
  for (int i = 0; i < 3; i++) {
    parameter.t[i] = transformation[4 * i + 3];
    for (int j = 0; j < 3; j++) {
      parameter.r[i][j] = transformation[4 * i + j];
    }
  }
  
  // Copy parameters to device
  cudaError err = cudaMemcpy(parameter_devptr, &parameter, sizeof(LidarParameter),
                              cudaMemcpyHostToDevice);
  if (err != cudaSuccess) {
    printf("CUDA Error: Failed to copy parameters to device\n");
    return;
  }
  
  // Allocate device memory for range output
  int total_bins = parameter.horizontal_bins * parameter.vertical_bins;
  float* dev_range_ptr;
  err = cudaMalloc(&dev_range_ptr, total_bins * sizeof(float));
  if (err != cudaSuccess) {
    printf("CUDA Error: Failed to allocate range buffer\n");
    return;
  }
  
  // Initialize range buffer
  dim3 init_block(256);
  dim3 init_grid((total_bins + init_block.x - 1) / init_block.x);
  lidar_range_init<<<init_grid, init_block>>>(dev_range_ptr, total_bins);
  
  // Render LiDAR scan
  dim3 render_block(256);
  dim3 render_grid((cloud_size + render_block.x - 1) / render_block.x);
  lidar_render<<<render_grid, render_block>>>(dev_cloud_ptr, parameter_devptr, dev_range_ptr);
  
  // Copy result back to host
  err = cudaMemcpy(host_range_ptr, dev_range_ptr, total_bins * sizeof(float),
                   cudaMemcpyDeviceToHost);
  if (err != cudaSuccess) {
    printf("CUDA Error: Failed to copy range data to host\n");
  }
  
  cudaFree(dev_range_ptr);
}

