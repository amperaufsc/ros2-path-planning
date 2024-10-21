#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import numpy as np
from fs_msgs.msg import Track

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(Track, '/fsds/track', self.track_callback, 10)
        self.publisher_ = self.create_publisher(Path, 'path', 10)
        timer_period = 0.5  
        
    def track_callback(self, msg):
        pass
        
    def odom_callback(self, msg):
        self.get_logger().info('X: "%f"' % msg.pose.pose.position.x)
        self.get_logger().info('Y: "%f"' % msg.pose.pose.position.y)
        
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        yaw = np.arctan2(2*(orientation_q.w*orientation_q.z - orientation_q.x*orientation_q.y), 1 - 2*((orientation_q.y)**2 + (orientation_q.z)**2))
        self.get_logger().info('Yaw: "%f"' % yaw)
        
        path_msg = Path()
        
        poses = []
        pose = PoseStamped()
        
        pose.header = path_msg.header
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        
        poses.append(pose)
        
        pose = PoseStamped()
        
        pose.header = path_msg.header
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        
        poses.append(pose)
        path_msg.poses = poses
        self.publisher_.publish(path_msg)
        
        
def main():
    rclpy.init()
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()