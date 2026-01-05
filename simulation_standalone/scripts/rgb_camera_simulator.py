#!/usr/bin/env python3
"""
RGB Camera Simulator
Generates realistic RGB images from depth images using surface normal estimation
and simulated lighting effects.

Topics:
  Subscribed:
    - /uav_simulator/depth_image (sensor_msgs/Image): Depth image from camera
    - /uav_simulator/sensor_pose (geometry_msgs/TransformStamped): Camera pose in world frame
    
  Published:
    - /uav_simulator/rgb_image (sensor_msgs/Image): Simulated RGB image (BGR8)
    - /uav_simulator/gray_image (sensor_msgs/Image): Simulated grayscale image (MONO8)

This simulator creates realistic images by:
1. Computing surface normals from depth gradients
2. Applying Lambertian shading model
3. Adding ambient occlusion approximation
4. Optional noise and texture effects
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2


class RGBCameraSimulator:
    def __init__(self):
        rospy.init_node('rgb_camera_simulator', anonymous=True)
        
        # Camera intrinsics
        self.fx = rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/fx', 320.0)
        self.fy = rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/fy', 320.0)
        self.cx = rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/cx', 320.0)
        self.cy = rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/cy', 240.0)
        self.max_depth = rospy.get_param('/uav_model/sensing_parameters/max_depth', 5.0)
        self.min_depth = rospy.get_param('/uav_model/sensing_parameters/min_depth', 0.1)
        
        # RGB simulation parameters
        self.ambient_light = rospy.get_param('~ambient_light', 0.3)  # Ambient light intensity [0-1]
        self.diffuse_light = rospy.get_param('~diffuse_light', 0.7)  # Diffuse light intensity [0-1]
        self.light_direction = rospy.get_param('~light_direction', [0.0, 0.0, -1.0])  # Light from above
        self.add_noise = rospy.get_param('~add_noise', True)
        self.noise_std = rospy.get_param('~noise_std', 5.0)  # Noise standard deviation
        self.edge_enhancement = rospy.get_param('~edge_enhancement', 0.3)  # Edge enhancement factor
        
        # Color mode: 'gray', 'depth_colored', 'normal_colored', 'ambient_occlusion'
        self.color_mode = rospy.get_param('~color_mode', 'normal_colored')
        
        # Base color for walls/surfaces (BGR format)
        self.base_color = np.array(rospy.get_param('~base_color', [180, 180, 180]), dtype=np.uint8)
        
        self.bridge = CvBridge()
        
        # Normalize light direction
        self.light_dir = np.array(self.light_direction, dtype=np.float32)
        self.light_dir = self.light_dir / np.linalg.norm(self.light_dir)
        
        # Publishers
        self.rgb_pub = rospy.Publisher('/uav_simulator/rgb_image', Image, queue_size=1)
        self.gray_pub = rospy.Publisher('/uav_simulator/gray_image', Image, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/uav_simulator/depth_image', Image, self.depth_callback, queue_size=1)
        
        rospy.loginfo("[RGBCameraSimulator] Initialized")
        rospy.loginfo("  Color mode: %s", self.color_mode)
        rospy.loginfo("  Ambient: %.2f, Diffuse: %.2f", self.ambient_light, self.diffuse_light)
    
    def depth_callback(self, msg):
        """Process depth image and generate RGB image"""
        try:
            # Convert to numpy array
            if msg.encoding == '32FC1':
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding == '16UC1':
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth = depth.astype(np.float32) / 1000.0
            else:
                rospy.logwarn_once("[RGBCameraSimulator] Unknown encoding: %s", msg.encoding)
                return
            
            height, width = depth.shape
            
            # Create valid depth mask
            valid_mask = (depth > self.min_depth) & (depth < self.max_depth) & np.isfinite(depth)
            
            # Compute surface normals from depth gradients
            normals = self.compute_normals(depth, valid_mask)
            
            # Generate image based on color mode
            if self.color_mode == 'gray':
                rgb_image = self.generate_gray_shading(depth, normals, valid_mask)
            elif self.color_mode == 'depth_colored':
                rgb_image = self.generate_depth_colored(depth, valid_mask)
            elif self.color_mode == 'normal_colored':
                rgb_image = self.generate_normal_colored(normals, valid_mask)
            elif self.color_mode == 'ambient_occlusion':
                rgb_image = self.generate_ambient_occlusion(depth, normals, valid_mask)
            else:
                rgb_image = self.generate_gray_shading(depth, normals, valid_mask)
            
            # Add noise if enabled
            if self.add_noise:
                noise = np.random.normal(0, self.noise_std, rgb_image.shape).astype(np.float32)
                rgb_image = np.clip(rgb_image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
            
            # Publish RGB image
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
            rgb_msg.header = msg.header
            self.rgb_pub.publish(rgb_msg)
            
            # Also publish grayscale
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            gray_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
            gray_msg.header = msg.header
            self.gray_pub.publish(gray_msg)
            
        except Exception as e:
            rospy.logerr("[RGBCameraSimulator] Error: %s", str(e))
    
    def compute_normals(self, depth, valid_mask):
        """Compute surface normals from depth image using gradient method"""
        height, width = depth.shape
        
        # Compute 3D points
        u = np.arange(width)
        v = np.arange(height)
        u, v = np.meshgrid(u, v)
        
        # Back-project to 3D (camera frame)
        z = depth.copy()
        z[~valid_mask] = 0
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        # Compute gradients
        dzdx = cv2.Sobel(z, cv2.CV_32F, 1, 0, ksize=5) / 8.0
        dzdy = cv2.Sobel(z, cv2.CV_32F, 0, 1, ksize=5) / 8.0
        
        # Compute normals: n = (-dz/dx, -dz/dy, 1) normalized
        normals = np.zeros((height, width, 3), dtype=np.float32)
        normals[:, :, 0] = -dzdx * self.fx / z.clip(min=0.01)
        normals[:, :, 1] = -dzdy * self.fy / z.clip(min=0.01)
        normals[:, :, 2] = 1.0
        
        # Normalize
        norm = np.linalg.norm(normals, axis=2, keepdims=True)
        norm = np.clip(norm, 1e-6, None)
        normals = normals / norm
        
        # Set invalid regions to face camera
        normals[~valid_mask] = [0, 0, 1]
        
        return normals
    
    def generate_gray_shading(self, depth, normals, valid_mask):
        """Generate grayscale image using Lambertian shading"""
        height, width = depth.shape
        
        # Lambertian shading: I = ambient + diffuse * max(0, n · L)
        n_dot_l = np.sum(normals * self.light_dir, axis=2)
        n_dot_l = np.clip(n_dot_l, 0, 1)
        
        intensity = self.ambient_light + self.diffuse_light * n_dot_l
        intensity = np.clip(intensity, 0, 1)
        
        # Apply depth-based attenuation (far objects are slightly darker)
        depth_factor = 1.0 - 0.2 * (depth / self.max_depth)
        depth_factor = np.clip(depth_factor, 0.5, 1.0)
        intensity = intensity * depth_factor
        
        # Edge enhancement using Laplacian
        if self.edge_enhancement > 0:
            edges = cv2.Laplacian(depth, cv2.CV_32F)
            edges = np.abs(edges)
            edges = edges / (edges.max() + 1e-6)
            intensity = intensity - self.edge_enhancement * edges
            intensity = np.clip(intensity, 0, 1)
        
        # Convert to BGR
        gray = (intensity * 255).astype(np.uint8)
        rgb_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        # Apply base color tint
        rgb_image = (rgb_image.astype(np.float32) * self.base_color / 255.0).astype(np.uint8)
        
        # Set invalid regions to black
        rgb_image[~valid_mask] = [0, 0, 0]
        
        return rgb_image
    
    def generate_depth_colored(self, depth, valid_mask):
        """Generate depth-colored image using colormap"""
        # Normalize depth
        depth_normalized = (depth - self.min_depth) / (self.max_depth - self.min_depth)
        depth_normalized = np.clip(depth_normalized, 0, 1)
        depth_normalized = (depth_normalized * 255).astype(np.uint8)
        
        # Apply colormap (JET gives rainbow colors, TURBO is more perceptually uniform)
        rgb_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)
        
        # Set invalid regions to black
        rgb_image[~valid_mask] = [0, 0, 0]
        
        return rgb_image
    
    def generate_normal_colored(self, normals, valid_mask):
        """Generate normal-colored image (like a normal map visualization)"""
        # Map normals from [-1,1] to [0,255]
        # X: left-right (red), Y: up-down (green), Z: forward (blue)
        rgb_image = ((normals + 1.0) * 0.5 * 255).astype(np.uint8)
        
        # Convert from RGB to BGR for OpenCV
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        
        # Set invalid regions to neutral blue (facing camera)
        rgb_image[~valid_mask] = [255, 128, 128]  # BGR: blue tint
        
        return rgb_image
    
    def generate_ambient_occlusion(self, depth, normals, valid_mask):
        """Generate ambient occlusion approximation"""
        height, width = depth.shape
        
        # Use depth variance in local neighborhood as AO approximation
        kernel_size = 5
        depth_blur = cv2.GaussianBlur(depth, (kernel_size, kernel_size), 0)
        depth_var = cv2.GaussianBlur((depth - depth_blur) ** 2, (kernel_size * 2 + 1, kernel_size * 2 + 1), 0)
        
        # Normalize variance
        ao = np.sqrt(depth_var)
        ao = ao / (ao.max() + 1e-6)
        
        # Combine with basic shading
        n_dot_l = np.sum(normals * self.light_dir, axis=2)
        n_dot_l = np.clip(n_dot_l, 0, 1)
        
        intensity = self.ambient_light * (1.0 - ao * 0.5) + self.diffuse_light * n_dot_l
        intensity = np.clip(intensity, 0, 1)
        
        # Convert to BGR
        gray = (intensity * 255).astype(np.uint8)
        rgb_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        # Apply subtle color variation based on depth
        depth_normalized = (depth - self.min_depth) / (self.max_depth - self.min_depth)
        depth_normalized = np.clip(depth_normalized, 0, 1)
        
        # Warmer colors for close objects, cooler for far
        rgb_image[:, :, 0] = np.clip(rgb_image[:, :, 0] * (0.9 + 0.2 * depth_normalized), 0, 255).astype(np.uint8)
        rgb_image[:, :, 2] = np.clip(rgb_image[:, :, 2] * (1.1 - 0.2 * depth_normalized), 0, 255).astype(np.uint8)
        
        # Set invalid regions to black
        rgb_image[~valid_mask] = [0, 0, 0]
        
        return rgb_image


if __name__ == '__main__':
    try:
        node = RGBCameraSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

