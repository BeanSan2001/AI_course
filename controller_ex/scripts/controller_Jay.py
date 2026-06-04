#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Int32
from scout_msgs.msg import ScoutRCState
import math, time
from scipy.spatial.transform import Rotation as R
import numpy as np
from scout_msgs.msg import ScoutLightCmd
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.current_pose = None 
        self.distance_robot_human = 0.0  
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.human_state = 1 
        self.human_state_sub = self.create_subscription(
            Int32,
            'human_state',
            self.human_state_callback,
            10
        )
        self.rc_sub = self.create_subscription(ScoutRCState,'/scout_rc_state', self.rc_callback,5)
        self.follow_mode = 0
        self.last_command = "follow_back"
        self.target_distance = 1.0
        self.last_dist_change_time = 0.0  
        self.dist_change_cooldown = 1.5 
        self.backup_start_time = None
        
        # robot_mode 定義:
        # 0: Stop (主動煞車)
        # 1: Following (PID 跟隨)
        # 2: Dancing (跳舞)
        # 10: Navigation Passive (被動模式，不發布 cmd_vel，讓 Nav2 控制)
        self.robot_mode = 0 
        
        self.light_pub = self.create_publisher(
            ScoutLightCmd,
            '/light_control',
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.collisionsubscription = self.create_subscription(
            Bool,
            '/collision_state',
            self.collision_callback,
            10
        )

        self.genbot_subscription = self.create_subscription(
            String,
            '/genbot',
            self.genbot_callback,
            10
        )
        
        self.dance_mode = 0
        self.collision = False
        self.human_state = 1

    def collision_callback(self, msg: Bool):
            self.collision = msg.data

    def rc_callback(self, msg):
        """RC 遙控器介入邏輯"""
        if msg.swa == 0:     # 強制後跟隨
            self.robot_mode = 1
            self.follow_mode = 0
        elif msg.swa == 255: # 強制對齊
            self.robot_mode = 1
            self.follow_mode = 2
        elif msg.swa == 2:   # AI 託管 (恢復上一次指令)
            cmd = self.last_command
            
            if cmd == "transport":
                self.robot_mode = 10
            elif cmd == "back":
                self.robot_mode = 3
                self.backup_start_time = None
            elif cmd == "follow_side":
                self.robot_mode = 1; self.follow_mode = 2
            elif cmd == "follow_back" or cmd == "follow":
                self.robot_mode = 1; self.follow_mode = 0
            elif cmd == "dance1":
                self.robot_mode = 2; self.dance_mode = 0
            elif cmd == "dance2":
                self.robot_mode = 2; self.dance_mode = 1
            else:
                self.robot_mode = 0

    def genbot_callback(self, msg: String):
        """處理來自 Agent 的指令"""
        data = msg.data
        self.last_command = data
        self.get_logger().info(f"Genbot Cmd: {data}")
        current_time = self.get_clock().now().nanoseconds / 1e9
        if data in ["follow", "follow_back", "follow_side", "transport", "stop", "back"]:
            self.last_command = data

        if data == "dance1":
            self.robot_mode = 2; self.dance_mode = 0
        elif data == "dance2":
            self.robot_mode = 2; self.dance_mode = 1
        elif data == "follow":
            self.robot_mode = 1
        elif data == "stop":
            self.robot_mode = 0
        elif data == "back":
            self.robot_mode = 3
            self.backup_start_time = None
        elif data == "follow_back":
            self.robot_mode = 1; self.follow_mode = 0
        elif data == "follow_side":
            self.robot_mode = 1; self.follow_mode = 2
        elif data == "distance_plus":
            if (current_time - self.last_dist_change_time) > self.dist_change_cooldown:
                self.target_distance = min(self.target_distance + 0.5, 2.5)
                self.last_dist_change_time = current_time 
                self.get_logger().info(f"Distance INCREASED to: {self.target_distance}")
            else:
                self.get_logger().warn("Distance command ignored (Cooling down)")

        elif data == "distance_minus":
            if (current_time - self.last_dist_change_time) > self.dist_change_cooldown:
                self.target_distance = max(self.target_distance - 0.3, 0.4)
                self.last_dist_change_time = current_time  
                self.get_logger().info(f"Distance DECREASED to: {self.target_distance}")
            else:
                self.get_logger().warn("Distance command ignored (Cooling down)")
        elif data == "transport":
            self.robot_mode = 10
            self.light_off()

        self.get_logger().info(f"Received genbot message: {msg.data}, Current Mode: {self.robot_mode}")

    def human_state_callback(self, msg):
        self.human_state = msg.data

    
        
    @staticmethod
    def quaternion_to_yaw(quaternion):
        return R.from_quat(quaternion).as_euler('xyz')[2]
    
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> np.ndarray:
        return R.from_euler('xyz', [0.0, 0.0, yaw], degrees=False).as_quat()
    
    
    def get_transform(self, target_frame, source_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            return t
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform from {source_frame} to {target_frame}: {e}")
            return None

    def dancing1(self):
        # 搖頭次數
        NUM_SWINGS = 6
        SWING_ANGLE = math.radians(30)

        if not hasattr(self, 'dance_start_angle'):
            try:
                odom_tf = self.get_transform('odom', 'base_link')
                if odom_tf is not None:
                    q = odom_tf.transform.rotation
                    self.dance_start_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
                    self.dance_start_time = self.get_clock().now().nanoseconds / 1e9
                    self.dance_swing_idx = 0
                    self.dance_direction = 1
                    self.dance_waiting = False
                else:
                    return
            except Exception as e:
                self.get_logger().warn(f"Could not get odom for dancing: {e}")
                return
            
        self.cmd = Twist()
        self.cmd.linear.x = 0.0

        try:
            odom_tf = self.get_transform('odom', 'base_link')
            if odom_tf is not None:
                q = odom_tf.transform.rotation
                current_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
            else:
                return
        except Exception as e:
            self.get_logger().warn(f"Could not get odom for dancing: {e}")
            return

        if self.dance_swing_idx < NUM_SWINGS:
            if not self.dance_waiting:
                offset = SWING_ANGLE * self.dance_direction
                self.dance_target_angle = self.dance_start_angle + offset
                self.dance_target_angle = (self.dance_target_angle + math.pi) % (2 * math.pi) - math.pi
                self.dance_waiting = True

            angle_error = (self.dance_target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

            if abs(angle_error) > 0.03:
                self.blink_light()
                self.cmd.angular.z = max(min(3.0 * angle_error, 2.0), -2.0)
                self.cmd_pub.publish(self.cmd)
            else:
                self.dance_direction *= -1
                self.dance_swing_idx += 1
                self.dance_waiting = False
        else:
            angle_error = (self.dance_start_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_error) > 0.03:
                self.cmd.angular.z = max(min(2.0 * angle_error, 0.7), -0.7)
                self.cmd_pub.publish(self.cmd)
            else:
                self.robot_mode = 0
                self.last_command = "stop"
                for attr in ['dance_start_angle', 'dance_start_time', 'dance_swing_idx', 'dance_direction', 'dance_waiting', 'dance_target_angle']:
                    if hasattr(self, attr):
                        delattr(self, attr)

    def dancing2(self):
        ROTATE_DURATION = 10.0
        ROTATE_SPEED = 2.5

        if not hasattr(self, 'dancing2_start_time'):
            self.dancing2_start_time = self.get_clock().now().nanoseconds / 1e9
            odom_tf = self.get_transform('odom', 'base_link')
            if odom_tf is not None:
                q = odom_tf.transform.rotation
                self.dancing2_start_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
            else:
                self.dancing2_start_angle = 0.0

        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.dancing2_start_time

        self.cmd = Twist()
        if elapsed < ROTATE_DURATION:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = ROTATE_SPEED
            self.cmd_pub.publish(self.cmd)
            self.blink_light()
        else:
            odom_tf = self.get_transform('odom', 'base_link')
            if odom_tf is not None:
                q = odom_tf.transform.rotation
                current_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
                angle_error = (self.dancing2_start_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
                if abs(angle_error) > 0.03:
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = max(min(2.0 * angle_error, 0.7), -0.7)
                    self.cmd_pub.publish(self.cmd)
                    return
            self.robot_mode = 0
            self.last_command = "stop"
            for attr in ['dancing2_start_time', 'dancing2_start_angle']:
                if hasattr(self, attr):
                    delattr(self, attr)

    def light_off(self):
        light_cmd = ScoutLightCmd()
        light_cmd.cmd_ctrl_allowed = True
        light_cmd.front_mode = 0
        light_cmd.front_custom_value = 0
        light_cmd.rear_mode = 0
        light_cmd.rear_custom_value = 0
        self.light_pub.publish(light_cmd)

    def blink_light(self):
        light_cmd = ScoutLightCmd()
        light_cmd.cmd_ctrl_allowed = True
        light_cmd.front_mode = 2
        light_cmd.front_custom_value = 100
        light_cmd.rear_mode = 2
        light_cmd.rear_custom_value = 100
        self.light_pub.publish(light_cmd)
    
    def move_backward(self):
        """
        機器人以固定速度後退 1 公尺。
        速度: -0.2 m/s
        時間: 5.0 秒
        """
        SPEED = -0.2
        TARGET_DISTANCE = 2.0
        DURATION = TARGET_DISTANCE / abs(SPEED)

        self.cmd = Twist()

        # 初始化開始時間
        if self.backup_start_time is None:
            self.backup_start_time = self.get_clock().now().nanoseconds / 1e9

        elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - self.backup_start_time

        if elapsed_time < DURATION:
            self.cmd.linear.x = SPEED
            self.cmd.angular.z = 0.0
            self.cmd_pub.publish(self.cmd)
            self.blink_light() # 後退時閃燈警示
        else:
            self.robot_mode = 0
            self.cmd.linear.x = 0.0
            self.cmd_pub.publish(self.cmd)
            self.light_off()
            self.backup_start_time = None # 清除計時器

    def following_mode(self):
        # 1. 取得座標 Transform
        right_tf = self.get_transform('base_link', 'right_following')
        left_tf = self.get_transform('base_link', 'left_following')
        center_tf = self.get_transform('base_link', 'human_link')
        self.cmd = Twist()

        # 計算左右距離 (若 TF 存在)
        left_dist = None
        right_dist = None
        if left_tf is not None:
            left_dist = math.hypot(left_tf.transform.translation.x, left_tf.transform.translation.y)
        if right_tf is not None:
            right_dist = math.hypot(right_tf.transform.translation.x, right_tf.transform.translation.y)

        # 2. 選擇目標 TF 與設定偏移距離
        chosen_tf = None
        offset_distance = 0.0

        if self.follow_mode > 0:  # 側邊跟隨模式
            if left_dist is not None and right_dist is not None:
                # 選擇較近的一側作為目標
                if left_dist <= right_dist:
                    chosen_tf = left_tf
                else:
                    chosen_tf = right_tf
            elif left_dist is not None:
                chosen_tf = left_tf
            elif right_dist is not None:
                chosen_tf = right_tf
            offset_distance = self.target_distance # 使用動態設定的距離
        else:                     # 中心後跟隨模式
            chosen_tf = center_tf
            offset_distance = self.target_distance # 使用動態設定的距離

        # 3. 簡單比例控制 (P-Control) 與發布速度
        if self.collision == False:
            if chosen_tf is not None and self.human_state == 1:
                # 取得基礎誤差資訊
                error_x = chosen_tf.transform.translation.x
                error_y = chosen_tf.transform.translation.y
                
                # 計算距離與相對角度
                distance = math.hypot(error_x, error_y)
                relative_angle = math.atan2(error_y, error_x)

                # --- 簡單 PID---
                # Linear X 控制:
                kp_lin = 1.3
                linear_x = kp_lin * (distance - offset_distance)
                linear_x = max(min(linear_x, 2.0), -1.5) # 限制最大最小速度

                # Angular Z 控制:
                kp_ang = 1.2
                angular_z = kp_ang * relative_angle
                angular_z = max(min(angular_z, 1.0), -1.0) # 限制最大最小轉速
                # ----------------------------------

                # 安全防護: 如果目標在機器人後方很近 (x < -0.5)，停止後退
                if error_x < -0.5:
                    linear_x = 0.0

                # 轉向安全防護: 如果正在大角度轉彎，則停止前進 (避免甩尾)
                if abs(angular_z) > 0.7 and abs(linear_x) > 0.7:
                    linear_x *= 0.3
                    angular_z*= 1.5

                # 發布指令
                self.cmd.linear.x = linear_x
                self.cmd.angular.z = angular_z
                self.cmd_pub.publish(self.cmd)
                
            else:
                # 沒看到人、或者 human_state == 0
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.cmd_pub.publish(self.cmd)
                
        else:
            # 發生碰撞，強制停止
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.cmd_pub.publish(self.cmd)
    # def following_mode(self):
    #     right_tf = self.get_transform('base_link', 'right_following')
    #     left_tf = self.get_transform('base_link', 'left_following')
    #     center_tf = self.get_transform('base_link', 'human_link')
    #     self.cmd = Twist()

    #     left_dist = None
    #     right_dist = None
    #     if left_tf is not None:
    #         left_x = left_tf.transform.translation.x
    #         left_y = left_tf.transform.translation.y
    #         left_dist = math.hypot(left_x, left_y)
    #     if right_tf is not None:
    #         right_x = right_tf.transform.translation.x
    #         right_y = right_tf.transform.translation.y
    #         right_dist = math.hypot(right_x, right_y)
    #     if center_tf is not None:
    #         center_x = center_tf.transform.translation.x
    #         center_y = center_tf.transform.translation.y
    #         center_dist = math.hypot(center_x, center_y)

    #     chosen_tf = None

    #     if self.follow_mode > 0:
    #         if left_dist is not None and right_dist is not None:
    #             if left_dist <= right_dist:
    #                 chosen_tf = left_tf
    #             else:
    #                 chosen_tf = right_tf
    #         elif left_dist is not None:
    #             chosen_tf = left_tf
    #         elif right_dist is not None:
    #             chosen_tf = right_tf
    #         offset_distane = 0.0
    #     else:
    #         chosen_tf = center_tf
    #         offset_distane = 1.0

    #     if self.collision==False:
    #         if chosen_tf is not None:
    #             error_x = chosen_tf.transform.translation.x
    #             error_y = chosen_tf.transform.translation.y
    #             distance = math.hypot(error_x, error_y)
    #             linear_x = self.PID(distance - offset_distane, 1.0)
    #             linear_x = max(min(linear_x, 2.0), -1.5)
    #             relative_angle = math.atan2(error_y, error_x)
    #             if error_x<-0.5:
    #                 linear_x = 0.0
    #             angular_z = self.PID(relative_angle, 2.0)
    #             angular_z = max(min(angular_z, 1.0), -1.0)

    #             if abs(angular_z)>0.7 and abs(linear_x)>0.7:
    #                 linear_x = 0.0

    #             if self.human_state == 1:
    #                 self.cmd.linear.x = linear_x
    #                 self.cmd.angular.z = angular_z
    #                 self.cmd_pub.publish(self.cmd)
    #             elif self.human_state == 0:
    #                 self.cmd.linear.x = 0.0
    #                 self.cmd.angular.z = 0.0
    #                 self.cmd_pub.publish(self.cmd)

    #     else:
    #         self.cmd.linear.x = 0.0
    #         self.cmd.angular.z = 0.0
    #         self.cmd_pub.publish(self.cmd)


    def timer_callback(self):
        if self.robot_mode == 10:
            return
        
        elif self.robot_mode == 3:      # 後退模式
            self.move_backward()

        # 模式 2: 跳舞
        elif self.robot_mode == 2:
            if self.dance_mode == 0:
                self.dancing1()
            else:
                self.dancing2()
        
        # 模式 1: 跟隨
        elif self.robot_mode == 1:
            self.light_off()
            self.following_mode()
        
        # 模式 0: 停止 (主動發布 0 速度以鎖定馬達)
        else:
            self.light_off()
            self.cmd = Twist()
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.cmd_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()