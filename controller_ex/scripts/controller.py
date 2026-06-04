#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Int32, Bool

class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower_node')

        # 速度控制指令的發布
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 設定 TF2 監聽器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 訂閱人類狀態 (0: 停止, 1: 跟隨)
        self.human_state = 1
        self.human_state_sub = self.create_subscription(
            Int32, 'human_state', self.human_state_callback, 10
        )

        self.timer = self.create_timer(0.001, self.timer_callback)

    def human_state_callback(self, msg: Int32):
        """更新跟隨狀態開關"""
        self.human_state = msg.data

    def PID(self, error, Kp):
        """比例 (P) 控制器，輸出與誤差成正比"""
        return Kp * error

    def get_transform(self, target_frame, source_frame):
        """獲取兩個座標系"""
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            return t
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def following_mode(self):
        """跟隨邏輯"""
        center_tf = self.get_transform('base_link', 'human_link')
        self.cmd = Twist()

        # 安全檢查：若無碰撞風險才計算速度
        if not self.collision:
            if center_tf is not None:
                error_x = center_tf.transform.translation.x
                error_y = center_tf.transform.translation.y
                distance = math.hypot(error_x, error_y)

                # PID 控制
                linear_x = self.PID(distance - 1.0, 1.0)
                linear_x = max(min(linear_x, 2.0), -1.5)  # 限制最高與最低速度

                relative_angle = math.atan2(error_y, error_x)
                angular_z = self.PID(relative_angle, 2.0)
                angular_z = max(min(angular_z, 1.0), -1.0) # 限制最大旋轉速度

                if error_x < -0.5:
                    linear_x = 0.0

                if abs(angular_z) > 0.7 and abs(linear_x) > 0.7:
                    linear_x = 0.0

                if self.human_state == 1:
                    self.cmd.linear.x = linear_x
                    self.cmd.angular.z = angular_z

        # 發布指令
        self.cmd_pub.publish(self.cmd)

    def timer_callback(self):
        """Timer 定時觸發，不斷執行跟隨模式"""
        self.following_mode()

def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()