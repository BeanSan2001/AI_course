#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from zed_msgs.msg import ObjectsStamped
from rclpy.time import Duration
from scout_msgs.msg import ScoutRCState
from gimbal_msg.msg import MotorCmd, MotorState
import math

class GimbalTrackingSimplified(Node):
    def __init__(self):
        super().__init__('gimbal_tracking')
        
        # --- 計時器與基本設定 ---
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.init_mode = 0  # 0: 停止並回正, 2: 跟隨模式
        
        # --- 訂閱者 (Subscribers) ---
        # 1. 訂閱 ZED 相機的骨架資訊
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/zed_multi/zed_gimbal/body_trk/skeletons',
            self.skeletons_callback,
            10)
        
        # 2. 訂閱遙控器與 Genbot 指令
        self.rc_sub = self.create_subscription(ScoutRCState, '/scout_rc_state', self.rc_callback, 5)
        self.genbot_sub = self.create_subscription(String, '/genbot', self.genbot_callback, 10)
        
        # 3. 訂閱雲台馬達編碼器狀態
        self.encoder_subscriber = self.create_subscription(
            MotorState, 'gimbal_state', self.encoder_callback, 10
        )

        # --- 發布者 (Publishers) ---
        # 控制雲台馬達電壓的發布器
        self.gimbal_pub = self.create_publisher(MotorCmd, 'gimbal_control', 10)
        # 用於發布人類 TF 座標
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- 狀態變數 ---
        self.human = None
        self.detect_human = False
        self.locked_id = None
        self.min_tracking = 2.0 
        self.encoder_value = 0.0
        self.last_command = "follow"
        self.last_lost_time = None
        self.reacquire_timeout = Duration(seconds=15.0)

        # --- 追蹤控制參數 ---
        self.image_width = self.declare_parameter('image_width', 1920).value
        self.image_center_x = self.image_width / 2.0
        self.filtered_output = 0.0  
        self.alpha = 0.1 
        self.camera_frame = "zed_gimbal_left_camera_frame"
        self.human_frame = "human_link"

    # ==========================================
    # 回呼函數 (Callbacks)
    # ==========================================

    def rc_callback(self, msg):
        """處理遙控器訊號，切換停止與跟隨模式"""
        if msg.swd == 0:
            self.init_mode = 0  # 停止
        elif msg.swd == 2:
            if self.last_command == "follow":
                self.init_mode = 2  # 跟隨
            else:
                self.init_mode = 0  # 預設停止
        else:
            self.init_mode = msg.swd

    def genbot_callback(self, msg):
        """處理外部字串指令 (如：follow, stop)"""
        valid_commands = ["follow", "stop"]
        if msg.data in valid_commands:
            self.last_command = msg.data
            if msg.data == "stop":
                self.init_mode = 0
            elif msg.data == "follow":
                self.init_mode = 2

    def encoder_callback(self, msg):
        """更新雲台目前的角度"""
        self.encoder_value = msg.angle_deg

    def skeletons_callback(self, msg):
        """處理人體骨架資料，決定鎖定與跟隨哪個人"""
        closest_human = None
        min_distance = float('inf')
        max_tracking_distance = 5.0
        reacquire_distance = 2.0

        now = self.get_clock().now()

        # 如果丟失目標超過 15 秒，重新尋找新目標
        if self.locked_id is not None and not self.detect_human:
            if self.last_lost_time is None:
                self.last_lost_time = now
            elif now - self.last_lost_time > self.reacquire_timeout:
                self.get_logger().info("追蹤丟失過久，重新鎖定新目標")
                self.locked_id = None
                self.last_lost_time = None 
        else:
            self.last_lost_time = None 

        if self.locked_id is None:
            for obj in msg.objects:
                if not obj.tracking_available:
                    continue
                distance = np.linalg.norm(obj.position[0:2])
                if distance < self.min_tracking and distance < min_distance and distance < reacquire_distance:
                    min_distance = distance
                    closest_human = obj

            if closest_human is not None:
                self.human = closest_human
                self.locked_id = closest_human.label_id
                self.detect_human = True
                self.get_logger().info(f"已鎖定目標 ID: {self.locked_id}")
            else:
                self.detect_human = False
        else:
            locked_position = np.array(self.human.position[0:2])
            for obj in msg.objects:
                if not obj.tracking_available:
                    continue
                nearest_distance = np.linalg.norm(np.array(obj.position[0:2]) - locked_position)
                distance = np.linalg.norm(obj.position[0:2])
                if nearest_distance < min_distance and distance < max_tracking_distance:
                    min_distance = nearest_distance
                    closest_human = obj

            if closest_human is not None:
                self.human = closest_human
                self.detect_human = True
            else:
                self.detect_human = False
                self.get_logger().info("目標追蹤丟失")

    # ==========================================
    # 核心邏輯與控制
    # ==========================================

    def publish_human_tf(self):
        """發布被鎖定人類相對於相機的 TF 座標"""
        if self.human is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.human_frame

        t.transform.translation.x = float(self.human.position[0])
        t.transform.translation.y = float(self.human.position[1])
        t.transform.translation.z = float(self.human.position[2])

        quat = self.human.global_root_orientation
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    def get_2d_center_x(self, human_obj):
        """計算 2D 邊界框 (Bounding Box) 的 X 軸中心點"""
        corners = human_obj.bounding_box_2d.corners
        x_sum = sum([c.kp[0] for c in corners])
        return x_sum / 4.0

    def timer_callback(self):
        """主控制迴圈 (1000Hz)，負責發送雲台控制訊號"""
        gimbal_control = MotorCmd()

        if self.init_mode == 0:
            # --- 停止模式：雲台自動回正到 0 度 ---
            Kp = 1.3
            Kd = 0.001
            setpoint = 0.0 
            
            error = setpoint - self.encoder_value
            error = (error + math.pi) % (2 * math.pi) - math.pi
            
            if not hasattr(self, 'last_error'):
                self.last_error = 0.0
            derivative = error - self.last_error
            self.last_error = error
            
            # 簡單的 PD 控制器計算電壓
            control = Kp * error + Kd * derivative
            gimbal_control.voltage = float(control)
            self.gimbal_pub.publish(gimbal_control)

            # 重置目標鎖定
            self.detect_human = False
            self.locked_id = None
            return

        elif self.init_mode == 2:
            # --- 跟隨模式：根據畫面中心與目標中心的像素誤差進行控制 ---
            if self.detect_human and self.human is not None:
                target_x = self.get_2d_center_x(self.human)
                pixel_error = self.image_center_x - target_x
                normalized_error = pixel_error / self.image_center_x
                
                # 低通濾波，使雲台移動更平穩
                self.filtered_output = self.alpha * normalized_error + (1 - self.alpha) * self.filtered_output
                control = float(3.5 * self.filtered_output)
                
                gimbal_control.voltage = control
                self.gimbal_pub.publish(gimbal_control)
                self.publish_human_tf()

            elif not self.detect_human and self.human is not None:
                # 剛丟失目標時，給予雲台微小轉向動力，嘗試找回目標
                gimbal_control.voltage = 1.5
                self.gimbal_pub.publish(gimbal_control)
                self.publish_human_tf()
            else:
                # 完全找不到目標，雲台靜止
                gimbal_control.voltage = 0.0
                self.gimbal_pub.publish(gimbal_control)
                if self.human is not None:
                    self.publish_human_tf()

def main(args=None):
    rclpy.init(args=args)
    try:
        gimbal_tracking = GimbalTrackingSimplified()
        rclpy.spin(gimbal_tracking)
    except KeyboardInterrupt:
        print("正在關閉節點...")
    except Exception as e:
        print(f"發生錯誤: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()