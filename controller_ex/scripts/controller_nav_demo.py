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
        self.current_pose = None  # 儲存最新的姿態
        self.distance_robot_human = 0.0  
        # 建立計時器，每 0.001 秒 (1000Hz) 發送一次控制指令
        self.timer = self.create_timer(0.001, self.timer_callback)
        # TF2 設定
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.human_state = 1  # 儲存人類狀態 human_state (0 或 1)
        self.human_state_sub = self.create_subscription(
            Int32,
            'human_state',
            self.human_state_callback,
            10
        )
        # self.rc_sub = self.create_subscription(ScoutRCState,'/scout_rc_state', self.rc_callback,5)
        self.follow_mode = 0
        self.robot_mode = 0  # 0: 停止, 1: 跟隨, 2: 跳舞
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
        self.saved_pose = None
        self.genbot_msg = None
        self.current_goal = None
        self.goal_tolerance = 0.25  # 判斷到達目標的距離閾值 (Threshold)

    def collision_callback(self, msg: Bool):
        self.collision = msg.data

    def genbot_callback(self, msg: String):
        self.genbot_msg = msg.data
        if msg.data == "dance1":
            self.robot_mode = 2
            self.dance_mode = 0
        elif msg.data == "dance2":
            self.robot_mode = 2
            self.dance_mode = 1
        elif msg.data == "follow":
            self.robot_mode = 1
        elif msg.data == "stop":
            self.robot_mode = 4
        elif msg.data == "find_tool":
            self.robot_mode = 3
            self.current_goal = [6.95, 2.48, 0.87]
            self.send_goal(self.current_goal[0], self.current_goal[1], self.current_goal[2])
        elif msg.data == "hungry":
            self.robot_mode = 3
            self.current_goal = [7.8, -0.925, 0.0]
            self.send_goal(self.current_goal[0], self.current_goal[1], self.current_goal[2])        
            
        self.get_logger().info(f"Received genbot message: {msg.data}")

    def human_state_callback(self, msg):
        self.human_state = msg.data
        # self.get_logger().info(f"Received human_state: {self.human_state}")
        
    
    @staticmethod
    def quaternion_to_yaw(quaternion):
        """從四元數提取 Yaw 角度。"""
        return R.from_quat(quaternion).as_euler('xyz')[2]
    
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> np.ndarray:
        """將 Yaw 角度 (弧度) 轉換為四元數。"""
        return R.from_euler('xyz', [0.0, 0.0, yaw], degrees=False).as_quat()
    

    def PID(self, error, Kp):
        output = Kp * error
        return output
    
    def get_transform(self, target_frame, source_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            # self.get_logger().info(
            #     f"Transform from {source_frame} to {target_frame}: "
            #     f"translation=({t.transform.translation.x}, {t.transform.translation.y}, {t.transform.translation.z})"
            # )
            return t
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform from {source_frame} to {target_frame}: {e}")
            return None

   
    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        # 多次發布以確保 Nav2 接收到
        for _ in range(5):
            self.publisher.publish(msg)

        self.get_logger().info(
            f"Published goal: x={x}, y={y}, yaw={yaw}"
        )

    def rc_callback(self, msg):
      self.follow_mode = msg.swa

    def dancing1(self):
        # 搖頭次數 (每次向左/右算一次)
        NUM_SWINGS = 6
        # 每次偏轉角度 (弧度)
        SWING_ANGLE = math.radians(30)  # 每側 30 度

        if not hasattr(self, 'dance_start_angle'):
            try:
                odom_tf = self.get_transform('odom', 'base_link')
                if odom_tf is not None:
                    q = odom_tf.transform.rotation
                    self.dance_start_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
                    self.dance_start_time = self.get_clock().now().nanoseconds / 1e9
                    self.dance_swing_idx = 0
                    self.dance_direction = 1  # 1: 右, -1: 左
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

        # 為每次搖擺產生目標角度
        if self.dance_swing_idx < NUM_SWINGS:
            if not self.dance_waiting:
                # 確定此次搖擺的目標角度
                offset = SWING_ANGLE * self.dance_direction
                self.dance_target_angle = self.dance_start_angle + offset
                # 正規化至 [-pi, pi]
                self.dance_target_angle = (self.dance_target_angle + math.pi) % (2 * math.pi) - math.pi
                self.dance_waiting = True

            # 計算角度誤差
            angle_error = (self.dance_target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

            if abs(angle_error) > 0.03:
                self.blink_light()
                self.cmd.angular.z = max(min(3.0 * angle_error, 2.0), -2.0)
                self.cmd_pub.publish(self.cmd)
            else:
                # 已到達位置，切換方向進行下一次
                self.dance_direction *= -1
                self.dance_swing_idx += 1
                self.dance_waiting = False
        else:
            # 返回初始角度
            angle_error = (self.dance_start_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_error) > 0.03:
                self.cmd.angular.z = max(min(2.0 * angle_error, 0.7), -0.7)
                self.cmd_pub.publish(self.cmd)
            else:
                self.robot_mode = 0
                for attr in ['dance_start_angle', 'dance_start_time', 'dance_swing_idx', 'dance_direction', 'dance_waiting', 'dance_target_angle']:
                    if hasattr(self, attr):
                        delattr(self, attr)
    def dancing2(self):
        # 以固定角速度連續旋轉 10 秒，然後返回初始角度
        ROTATE_DURATION = 10.0  # 秒
        ROTATE_SPEED = 2.5     # 弧度/秒

        if not hasattr(self, 'dancing2_start_time'):
            self.dancing2_start_time = self.get_clock().now().nanoseconds / 1e9
            # 儲存初始角度
            odom_tf = self.get_transform('odom', 'base_link')
            if odom_tf is not None:
                q = odom_tf.transform.rotation
                self.dancing2_start_angle = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
            else:
                self.dancing2_start_angle = 0.0  # fallback

        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.dancing2_start_time

        self.cmd = Twist()
        if elapsed < ROTATE_DURATION:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = ROTATE_SPEED
            self.cmd_pub.publish(self.cmd)
            self.blink_light()
        else:
            # 返回初始角度
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
            # 結束 dancing2 模式
            self.robot_mode = 0
            for attr in ['dancing2_start_time', 'dancing2_start_angle']:
                if hasattr(self, attr):
                    delattr(self, attr)

    def light_off(self):
        light_cmd = ScoutLightCmd()
        light_cmd.cmd_ctrl_allowed = True
        light_cmd.front_mode = 0  # LIGHT_CONST_OFF
        light_cmd.front_custom_value = 0
        light_cmd.rear_mode = 0   # LIGHT_CONST_OFF
        light_cmd.rear_custom_value = 0
        self.light_pub.publish(light_cmd)

    def blink_light(self):
        light_cmd = ScoutLightCmd()
        light_cmd.cmd_ctrl_allowed = True
        light_cmd.front_mode = 2  # LIGHT_BLINK
        light_cmd.front_custom_value = 100  # 閃爍頻率
        light_cmd.rear_mode = 2   # LIGHT_BLINK
        light_cmd.rear_custom_value = 100
        self.light_pub.publish(light_cmd)


    def following_mode(self):
        right_tf = self.get_transform('base_link', 'right_following')
        left_tf = self.get_transform('base_link', 'left_following')
        center_tf = self.get_transform('base_link', 'human_link')
        self.cmd = Twist()

        # 如果有座標變換，計算距離
        left_dist = None
        right_dist = None
        if left_tf is not None:
            left_x = left_tf.transform.translation.x
            left_y = left_tf.transform.translation.y
            left_dist = math.hypot(left_x, left_y)
        if right_tf is not None:
            right_x = right_tf.transform.translation.x
            right_y = right_tf.transform.translation.y
            right_dist = math.hypot(right_x, right_y)
        if center_tf is not None:
            center_x = center_tf.transform.translation.x
            center_y = center_tf.transform.translation.y
            center_dist = math.hypot(center_x, center_y)

        # 選擇最近的座標變換
        chosen_tf = None

        if self.follow_mode > 0:
            if left_dist is not None and right_dist is not None:
                if left_dist <= right_dist:
                    chosen_tf = left_tf
                else:
                    chosen_tf = right_tf
            elif left_dist is not None:
                chosen_tf = left_tf
            elif right_dist is not None:
                chosen_tf = right_tf
            offset_distane = 0.0
        else:
            chosen_tf = center_tf
            offset_distane = 1.0

        # 根據最近的座標變換控制機器人
        if self.collision==False:
            if chosen_tf is not None:
                error_x = chosen_tf.transform.translation.x
                error_y = chosen_tf.transform.translation.y
                distance = math.hypot(error_x, error_y)
                linear_x = self.PID(distance - offset_distane, 1.0)  # 保持 1.0m 距離 (可修改)
                linear_x = max(min(linear_x, 2.0), -1.5)
                relative_angle = math.atan2(error_y, error_x)
                if error_x<-0.5:
                    linear_x = 0.0
                # print(relative_angle)
                angular_z = self.PID(relative_angle, 2.0)
                angular_z = max(min(angular_z, 1.0), -1.0)

                if abs(angular_z)>0.7 and abs(linear_x)>0.7:
                    linear_x = 0.0

                if self.human_state == 1:
                    self.cmd.linear.x = linear_x
                    self.cmd.angular.z = angular_z
                    self.cmd_pub.publish(self.cmd)
                elif self.human_state == 0:
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd)

        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.cmd_pub.publish(self.cmd)


    def timer_callback(self):
        # print(self.robot_mode,self.dance_mode)
        # 選擇運作模式: 跳舞、跟隨或停止
        if self.robot_mode == 2:
            # 跳舞情況
            if self.dance_mode ==0:
                self.dancing1()
            else:
                self.dancing2()
        elif self.robot_mode == 1:
            self.light_off()
            self.following_mode()
        elif self.robot_mode == 3:
            odom_tf = self.get_transform('map', 'base_link')
            if odom_tf is not None:
                x = odom_tf.transform.translation.x
                y = odom_tf.transform.translation.y
                # 從四元數取得 Yaw
                q = odom_tf.transform.rotation
                yaw = self.quaternion_to_yaw([q.x, q.y, q.z, q.w])
                self.saved_pose = [x, y, yaw]
                self.robot_mode = 5
        elif self.robot_mode == 5:      
            if self.current_goal is not None:
                x_goal, y_goal, _ = self.current_goal
                odom_tf = self.get_transform('map', 'base_link')  # 當前姿態
                x_robot = odom_tf.transform.translation.x
                y_robot = odom_tf.transform.translation.y
                distance = math.sqrt((x_goal - x_robot)**2 + (y_goal - y_robot)**2)
                if distance < self.goal_tolerance:
                    print("Goal reached!")
                    self.send_goal(self.saved_pose[0],self.saved_pose[1],self.saved_pose[2])
                    self.robot_mode = 6
                    self.current_goal = None  # 重置目標
            self.light_off()
            print("navigation 2 mode")
        elif self.robot_mode == 6:
            self.genbot_msg = None
            print("return mode")
        elif self.robot_mode == 4:
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