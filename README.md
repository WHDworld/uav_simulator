# UAV Simulator

A complete UAV simulation environment with depth camera and LiDAR sensors for ROS. This package can be used standalone for sensor testing, algorithm development, or integrated with planning/navigation systems.

## Features

- **Depth Camera Simulation**: GPU-accelerated depth image rendering using Open3D
- **LiDAR Simulation (CUDA)**: 360° Livox Mid-360 style point cloud generation
- **UAV Dynamics**: Simple position command to odometry converter
- **Interactive Control**: Use RViz "2D Nav Goal" to fly the UAV
- **Multiple Maps**: Pre-configured office, tunnel, and power plant environments

## Quick Start

### 1. Prerequisites

```bash
# ROS Noetic
sudo apt install ros-noetic-desktop-full

# CUDA Toolkit (for LiDAR simulation)
# Follow NVIDIA CUDA installation guide

# Open3D (for mesh rendering)
# Build from source or install via pip
pip3 install open3d

# PCL and OpenCV
sudo apt install libpcl-dev libopencv-dev
```

### 2. Build

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Run Simulation

```bash
# Basic simulation (depth camera + LiDAR)
roslaunch simulation_standalone uav_simulation.launch

# With specific map
roslaunch simulation_standalone uav_simulation.launch map_name:=classical_office

# Custom initial position
roslaunch simulation_standalone uav_simulation.launch init_x:=-6.0 init_y:=0.0 init_z:=1.5

# Disable LiDAR (for maps without .pcd)
roslaunch simulation_standalone uav_simulation.launch enable_lidar:=false

# Specify Open3D resource path
roslaunch simulation_standalone uav_simulation.launch open3d_resource_path:=/path/to/Open3D/build/bin/resources
```

## Topics

### Subscribed Topics (Control Input)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | Position command to control UAV |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 2D Nav Goal from RViz |

### Published Topics (Sensor Output)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/uav_simulator/odometry` | `nav_msgs/Odometry` | UAV ground truth odometry |
| `/uav_simulator/depth_image` | `sensor_msgs/Image` | Depth image (32FC1, meters) |
| `/uav_simulator/rgb_image` | `sensor_msgs/Image` | Simulated RGB image (BGR8) |
| `/uav_simulator/gray_image` | `sensor_msgs/Image` | Simulated grayscale image (MONO8) |
| `/uav_simulator/sensor_pose` | `geometry_msgs/TransformStamped` | Camera pose in world frame |
| `/depth_pointcloud` | `sensor_msgs/PointCloud2` | Depth camera point cloud (world frame) |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | LiDAR point cloud (world frame) |
| `/lidar/map` | `sensor_msgs/PointCloud2` | Full map point cloud (for visualization) |
| `/lidar/pose` | `geometry_msgs/TransformStamped` | LiDAR sensor pose |

### Visualization Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/odom_visualization/robot` | `visualization_msgs/Marker` | UAV mesh visualization |
| `/odom_visualization/pose` | `geometry_msgs/PoseStamped` | UAV pose for RViz |

## Control Interface

### Method 1: RViz 2D Nav Goal (Interactive)

1. Open RViz (automatically launched)
2. Click "2D Nav Goal" button in toolbar
3. Click and drag on the map to set position and heading
4. UAV will fly to the target location

### Method 2: Position Command (Programmatic)

Publish to `/planning/pos_cmd`:

```cpp
#include <quadrotor_msgs/PositionCommand.h>

quadrotor_msgs::PositionCommand cmd;
cmd.header.stamp = ros::Time::now();
cmd.header.frame_id = "world";

// Target position
cmd.position.x = target_x;
cmd.position.y = target_y;
cmd.position.z = target_z;

// Target velocity (optional, set to 0 for position control)
cmd.velocity.x = 0.0;
cmd.velocity.y = 0.0;
cmd.velocity.z = 0.0;

// Target yaw
cmd.yaw = target_yaw;  // radians

// Flags
cmd.trajectory_flag = 1;  // 1 = position control mode

pos_cmd_pub.publish(cmd);
```

Python example:

```python
#!/usr/bin/env python3
from quadrotor_msgs.msg import PositionCommand
import rospy

rospy.init_node('simple_control')
pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)

cmd = PositionCommand()
cmd.header.stamp = rospy.Time.now()
cmd.header.frame_id = 'world'
cmd.position.x = 0.0
cmd.position.y = 0.0
cmd.position.z = 1.0
cmd.yaw = 0.0
cmd.trajectory_flag = 1

pub.publish(cmd)
```

## Available Maps

| Map Name | Type | Has PCD | Description |
|----------|------|---------|-------------|
| `classical_office` | Indoor | ✅ | Small office environment |
| `darpa_tunnel` | Underground | ✅ | Tunnel exploration scenario |
| `power_plant` | Industrial | ✅ | Large industrial facility |
| `complex_office` | Indoor | ❌ | Complex office (STL only) |

**Note**: LiDAR simulation requires `.pcd` map files. Use `enable_lidar:=false` for maps without PCD.

## Sensor Parameters

### Depth Camera

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `fx`, `fy` | 320.0 | pixel | Focal length |
| `cx`, `cy` | 320.0, 240.0 | pixel | Principal point |
| `image_width` | 640 | pixel | Image width |
| `image_height` | 480 | pixel | Image height |
| `min_depth` | 0.1 | m | Minimum depth |
| `max_depth` | 5.0 | m | Maximum depth |
| `horizontal_fov` | 90° | - | Horizontal field of view |

### LiDAR (Livox Mid-360 style)

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `horizontal_fov` | 360° | - | Full 360° coverage |
| `vertical_fov` | -7° to +52° | - | 59° total vertical FOV |
| `min_range` | 0.1 | m | Minimum range |
| `max_range` | 30.0 | m | Maximum range |
| `target_points` | 10000 | - | Output point count |
| `render_rate` | 10.0 | Hz | Update rate |
| `range_noise_std` | 0.02 | m | Range noise (2cm) |

### RGB Camera Simulator

Generates realistic RGB images from depth using surface normal estimation and lighting simulation.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `color_mode` | `ambient_occlusion` | Rendering mode (see below) |
| `ambient_light` | 0.3 | Ambient light intensity [0-1] |
| `diffuse_light` | 0.7 | Diffuse light intensity [0-1] |
| `light_direction` | [0,0,-1] | Light direction vector |
| `add_noise` | true | Add Gaussian noise |
| `noise_std` | 3.0 | Noise standard deviation |
| `edge_enhancement` | 0.2 | Edge enhancement factor |

**Color Modes:**
- `gray`: Grayscale Lambertian shading
- `depth_colored`: Depth-based colormap (rainbow)
- `normal_colored`: Surface normal visualization (RGB = XYZ)
- `ambient_occlusion`: Realistic shading with AO approximation

## Package Structure

```
uav_simulator/
├── camera_sensing/          # Depth camera simulation (CUDA/Open3D)
│   ├── mesh_render/         # STL mesh rendering
│   └── pointcloud_render/   # PCD point cloud rendering
├── lidar_simulator/         # CUDA-accelerated LiDAR simulation
├── map_render/              # Map loading and rendering coordination
│   └── resource/            # Map files (.stl, .pcd)
├── poscmd_2_odom/           # Position command to odometry converter
├── simulation_standalone/   # Standalone launch files and configs
│   ├── config/
│   │   ├── uav_model.yaml   # UAV dynamics and sensor parameters
│   │   ├── render.yaml      # Render configuration
│   │   └── map/             # Map-specific configurations
│   ├── launch/
│   │   └── uav_simulation.launch
│   ├── scripts/
│   │   ├── waypoint_control.py
│   │   ├── depth_to_pointcloud.py
│   │   └── rgb_camera_simulator.py
│   └── rviz/
│       └── simulation.rviz
└── utils/                   # Utility packages
    ├── odom_visualization/  # Robot mesh visualizer
    ├── quadrotor_msgs/      # Custom message definitions
    └── ...
```

## Coordinate Frames

```
world (Fixed Frame, ENU: East-North-Up)
  │
  └── body (UAV body frame, FLU: Front-Left-Up)
       │
       ├── camera (Camera frame, z-forward, x-right, y-down)
       │
       └── lidar (LiDAR frame, x-forward, y-left, z-up)
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `map_name` | `classical_office` | Map to load |
| `enable_depth_camera` | `true` | Enable depth camera simulation |
| `enable_rgb_camera` | `true` | Enable RGB camera simulation |
| `enable_lidar` | `true` | Enable LiDAR simulation |
| `enable_depth_pointcloud` | `true` | Convert depth to pointcloud |
| `init_x`, `init_y`, `init_z` | -6.0, 0.0, 1.0 | Initial position |
| `init_yaw` | 0.0 | Initial yaw angle (rad) |
| `rviz` | `true` | Launch RViz visualization |
| `open3d_resource_path` | `~/install/Open3D/...` | Open3D resource path |
| `odom_topic` | `/uav_simulator/odometry` | Odometry output topic |
| `depth_image_topic` | `/uav_simulator/depth_image` | Depth image topic |
| `lidar_topic` | `/livox/lidar` | LiDAR pointcloud topic |
| `pos_cmd_topic` | `/planning/pos_cmd` | Position command topic |

## Dependencies

- ROS Noetic
- CUDA Toolkit (for LiDAR simulation)
- Open3D (for STL mesh rendering)
- PCL (Point Cloud Library)
- OpenCV
- Eigen3

## Troubleshooting

### LiDAR simulation not working

- Ensure map has `.pcd` file: `ls $(rospack find map_render)/resource/*.pcd`
- Check CUDA installation: `nvidia-smi`
- Use `enable_lidar:=false` if no PCD available

### Depth image not showing

- Check Open3D resource path in launch file
- Verify Open3D is installed: `python3 -c "import open3d; print(open3d.__version__)"`
- Check map file exists: `ls $(rospack find map_render)/resource/`

### UAV not moving

- Ensure you're publishing to `/planning/pos_cmd`
- Check odometry: `rostopic echo /uav_simulator/odometry`
- Use RViz "2D Nav Goal" for quick testing

### "fx, fy, cx, cy is empty!" error

- Make sure `uav_model.yaml` is loaded before `map_render_node`
- Check: `rosparam get /uav_model/sensing_parameters/camera_intrinsics/fx`

## Integration Example

To integrate with your own planning system:

```python
#!/usr/bin/env python3
"""
Example: Subscribe to sensor data and publish control commands
"""
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image
from quadrotor_msgs.msg import PositionCommand
import numpy as np

class MyPlanner:
    def __init__(self):
        rospy.init_node('my_planner')
        
        # Current state
        self.position = np.zeros(3)
        self.orientation = np.array([0, 0, 0, 1])  # [x, y, z, w]
        
        # Subscribe to sensor data
        rospy.Subscriber('/uav_simulator/odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/depth_pointcloud', PointCloud2, self.depth_pc_cb)
        rospy.Subscriber('/livox/lidar', PointCloud2, self.lidar_cb)
        rospy.Subscriber('/uav_simulator/depth_image', Image, self.depth_img_cb)
        
        # Publish control commands
        self.cmd_pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        
        rospy.loginfo("Planner initialized!")
    
    def odom_cb(self, msg):
        """Handle UAV odometry - get current position and orientation"""
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
    
    def depth_pc_cb(self, msg):
        """Handle depth camera point cloud - use for local obstacle avoidance"""
        # msg is sensor_msgs/PointCloud2
        # Convert to numpy array using ros_numpy or sensor_msgs.point_cloud2
        pass
    
    def lidar_cb(self, msg):
        """Handle LiDAR point cloud - use for mapping"""
        pass
    
    def depth_img_cb(self, msg):
        """Handle depth image - raw depth data"""
        # msg.encoding is '32FC1', depth in meters
        pass
    
    def send_command(self, x, y, z, yaw):
        """Send position command to UAV"""
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'world'
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0
        cmd.yaw = yaw
        cmd.trajectory_flag = 1
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        planner = MyPlanner()
        
        # Example: fly to a series of waypoints
        rate = rospy.Rate(1)  # 1 Hz
        waypoints = [
            (0.0, 0.0, 1.0, 0.0),
            (2.0, 0.0, 1.0, 0.0),
            (2.0, 2.0, 1.0, 1.57),
            (0.0, 2.0, 1.0, 3.14),
        ]
        
        for wp in waypoints:
            rospy.loginfo(f"Flying to {wp[:3]}")
            for _ in range(10):  # Send command for 10 seconds
                planner.send_command(*wp)
                rate.sleep()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

## Adding Custom Maps

1. Create map file (`.stl` for mesh or `.pcd` for point cloud):
```bash
# Place your map file in:
$(rospack find map_render)/resource/my_map.pcd
```

2. Create map configuration:
```yaml
# simulation_standalone/config/map/my_map.yaml
map_config:
  map_file: "my_map.pcd"
  map_dimension: 2
  
  init_x: 0.0
  init_y: 0.0
  init_z: 1.0
  init_yaw: 0.0

  map_size:
    map_min_x: -20.0
    map_min_y: -20.0
    map_min_z: -1.0
    map_max_x: 20.0
    map_max_y: 20.0
    map_max_z: 5.0

  scale: 1.0

  T_b_c:
  - [ 0.0, 0.0, 1.0, 0.0]
  - [-1.0, 0.0, 0.0, 0.0]
  - [ 0.0,-1.0, 0.0, 0.0]
  - [ 0.0, 0.0, 0.0, 1.0]

  T_m_w:
  - [ 1.0, 0.0, 0.0, 0.0]
  - [ 0.0, 1.0, 0.0, 0.0]
  - [ 0.0, 0.0, 1.0, 0.0]
  - [ 0.0, 0.0, 0.0, 1.0]
```

3. Launch with your map:
```bash
roslaunch simulation_standalone uav_simulation.launch map_name:=my_map
```

## License

MIT License
