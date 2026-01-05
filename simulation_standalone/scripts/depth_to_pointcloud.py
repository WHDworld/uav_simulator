#!/usr/bin/env python3
"""
Depth Image to PointCloud2 Converter
Subscribes to depth image and publishes PointCloud2 in world frame.

Topics:
  Subscribed:
    - /uav_simulator/depth_image (sensor_msgs/Image): Depth image from camera
    - /uav_simulator/sensor_pose (geometry_msgs/TransformStamped): Camera pose in world frame
    
  Published:
    - /depth_pointcloud (sensor_msgs/PointCloud2): Point cloud in world frame

Note: sensor_pose is T_w_c (camera pose in world frame)
Camera frame: z-forward, x-right, y-down (OpenCV convention)
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

class DepthToPointCloud:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud', anonymous=True)
        
        # Parameters
        self.fx = rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/fx', 320.0)
        self.fy = rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/fy', 320.0)
        self.cx = rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/cx', 320.0)
        self.cy = rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/cy', 240.0)
        self.max_depth = rospy.get_param('/uav_model/sensing_parameters/max_depth', 5.0)
        self.min_depth = rospy.get_param('/uav_model/sensing_parameters/min_depth', 0.1)
        
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.downsample = rospy.get_param('~downsample', 4)
        
        self.bridge = CvBridge()
        
        # Current sensor pose (camera in world frame)
        self.sensor_pose = None
        self.has_pose = False
        
        # Publishers
        self.pc_pub = rospy.Publisher('/depth_pointcloud', PointCloud2, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/uav_simulator/depth_image', Image, self.depth_callback)
        rospy.Subscriber('/uav_simulator/sensor_pose', TransformStamped, self.pose_callback)
        
        rospy.loginfo("[DepthToPointCloud] Initialized with fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", 
                      self.fx, self.fy, self.cx, self.cy)
        rospy.loginfo("[DepthToPointCloud] Publishing to /depth_pointcloud")
    
    def pose_callback(self, msg):
        """Store the current sensor pose (camera in world frame)"""
        self.sensor_pose = msg
        self.has_pose = True
    
    def depth_callback(self, msg):
        """Convert depth image to point cloud"""
        if not self.has_pose:
            return
        
        try:
            # Convert ROS Image to numpy array
            if msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            elif msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_image = depth_image.astype(np.float32) / 1000.0
            else:
                rospy.logwarn_once("[DepthToPointCloud] Unknown depth encoding: %s", msg.encoding)
                return
            
            height, width = depth_image.shape
            
            # Create pixel coordinate grids (downsampled)
            u = np.arange(0, width, self.downsample)
            v = np.arange(0, height, self.downsample)
            u, v = np.meshgrid(u, v)
            
            depth = depth_image[::self.downsample, ::self.downsample]
            
            # Filter valid depth values
            valid = (depth > self.min_depth) & (depth < self.max_depth) & np.isfinite(depth)
            
            # Convert to 3D points in camera frame (z-forward, x-right, y-down)
            z_cam = depth[valid]
            x_cam = (u[valid] - self.cx) * z_cam / self.fx
            y_cam = (v[valid] - self.cy) * z_cam / self.fy
            
            points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
            
            if len(points_cam) == 0:
                return
            
            # Get transformation from sensor pose (camera to world: T_w_c)
            t = self.sensor_pose.transform.translation
            q = self.sensor_pose.transform.rotation
            
            R_w_c = self.quat_to_rot(q.x, q.y, q.z, q.w)
            translation = np.array([t.x, t.y, t.z])
            
            # Transform points from camera frame to world frame
            points_world = (R_w_c @ points_cam.T).T + translation
            
            # Create PointCloud2 message
            header = msg.header
            header.frame_id = self.world_frame
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            pc_msg = pc2.create_cloud(header, fields, points_world)
            self.pc_pub.publish(pc_msg)
            
        except Exception as e:
            rospy.logerr("[DepthToPointCloud] Error: %s", str(e))
    
    def quat_to_rot(self, x, y, z, w):
        """Convert quaternion to rotation matrix"""
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])
        return R

if __name__ == '__main__':
    try:
        node = DepthToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

