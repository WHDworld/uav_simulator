#!/usr/bin/env python3
"""
Waypoint Control Node
Subscribes to 2D Nav Goal from RViz and publishes position commands to control the drone.

Topics:
  Subscribed:
    - /move_base_simple/goal (geometry_msgs/PoseStamped): 2D Nav Goal from RViz
    - /uav_simulator/odometry (nav_msgs/Odometry): Current UAV odometry
    
  Published:
    - /planning/pos_cmd (quadrotor_msgs/PositionCommand): Position command to UAV
"""

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
import math

class WaypointController:
    def __init__(self):
        rospy.init_node('waypoint_control', anonymous=True)
        
        # Parameters
        self.default_height = rospy.get_param('~default_height', 1.0)
        self.control_rate = rospy.get_param('~control_rate', 50.0)
        
        # Current state
        self.current_pos = [0.0, 0.0, 1.0]
        self.current_yaw = 0.0
        self.target_pos = None
        self.target_yaw = 0.0
        self.has_odom = False
        
        # Publishers
        self.cmd_pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/uav_simulator/odometry', Odometry, self.odom_callback)
        
        # Control timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_callback)
        
        rospy.loginfo("[WaypointControl] Initialized. Click '2D Nav Goal' in RViz to set waypoints.")
        rospy.loginfo("[WaypointControl] Default height: %.2f m", self.default_height)
    
    def odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.has_odom = True
        
        # Initialize target to current position if not set
        if self.target_pos is None:
            self.target_pos = self.current_pos.copy()
            self.target_yaw = self.current_yaw
    
    def goal_callback(self, msg):
        """Handle 2D Nav Goal from RViz"""
        self.target_pos = [
            msg.pose.position.x,
            msg.pose.position.y,
            self.default_height  # Use default height for 2D goals
        ]
        
        # Extract yaw from goal orientation
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.target_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        rospy.loginfo("[WaypointControl] New goal: (%.2f, %.2f, %.2f), yaw: %.2f rad",
                      self.target_pos[0], self.target_pos[1], self.target_pos[2], self.target_yaw)
    
    def control_callback(self, event):
        """Publish position command at fixed rate"""
        if not self.has_odom or self.target_pos is None:
            return
        
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'world'
        
        # Set target position
        cmd.position.x = self.target_pos[0]
        cmd.position.y = self.target_pos[1]
        cmd.position.z = self.target_pos[2]
        
        # Zero velocity and acceleration
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        
        cmd.yaw = self.target_yaw
        cmd.yaw_dot = 0.0
        
        cmd.trajectory_flag = 1  # Position control mode
        cmd.trajectory_id = 0
        
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        controller = WaypointController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

