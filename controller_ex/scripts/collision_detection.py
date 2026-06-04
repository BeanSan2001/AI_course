#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import sensor_msgs_py.point_cloud2 as pc2

class LidarCollisionChecker(Node):
    def __init__(self):
        super().__init__('lidar_collision_checker')

        # Subscribe to PointCloud2 topic
        # self.pointcloud_sub = self.create_subscription(
        #     PointCloud2, '/livox/lidar', self.pointcloud_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/velodyne_points', self.pointcloud_callback, 10)
        # Publisher for collision state
        self.collision_pub = self.create_publisher(Bool, '/collision_state', 10)
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.min_dist = 0.2
        self.max_dist = 0.3
        self.min_points = 5

    def pointcloud_callback(self, msg: PointCloud2):
        collision = False
        try:
            points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            cloud_points = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
            dists = np.linalg.norm(cloud_points, axis=1)
            mask = (dists > self.min_dist) & (dists < self.max_dist)
            num_points = np.count_nonzero(mask)

            if num_points > self.min_points:
                collision = True
                self.get_logger().warn("Warning: Collision detected!")
            else:
                self.get_logger().info("Safe: No collision detected.")

        except Exception as e:
            self.get_logger().error(f"Error reading PointCloud data: {e}")

        # Publish collision state
        self.collision_pub.publish(Bool(data=collision))

        # Stop robot if collision detected
        if collision:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCollisionChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
