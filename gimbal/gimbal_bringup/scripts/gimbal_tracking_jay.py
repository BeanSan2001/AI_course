#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
import serial
import numpy as np
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from zed_msgs.msg import ObjectsStamped, Object
from rclpy.time import Time, Duration
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from scout_msgs.msg import ScoutRCState
from gimbal_msg.msg import MotorCmd, MotorState
import math

class GimbalTracking(Node):
    def __init__(self):
        super().__init__('gimbal_tracking')
        # Declare parameters
      
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/zed_multi/zed_gimbal/body_trk/skeletons',
            self.skeletons_callback,
            10)
            
        self.sub_id = self.create_subscription(
            Int32, 
            '/locked_id', 
            self.id_cb, 
            3)

        self.human = None
        self.detect_human = False
        self.locked_id = -1 # 初始化為 -1 (未鎖定)
        
        self.min_tracking = 2.0  
        self.gimbal_pub = self.create_publisher(MotorCmd, 'gimbal_control', 10)
        self.filtered_output = 0.0  
        self.alpha = 0.1  
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_frame = "zed_gimbal_left_camera_frame"  
        self.human_frame = "human_link"
        
        self.marker_radius: float = self.declare_parameter('marker_radius', 0.5).value
        self.marker_height: float = self.declare_parameter('marker_height', 0.01).value
        self.marker_pub = self.create_publisher(Marker, '/human_marker', 10)
        self.semi_major_axis = self.declare_parameter('semi_major_axis', 1.1).get_parameter_value().double_value
        self.semi_minor_axis = self.declare_parameter('semi_minor_axis', 0.8).get_parameter_value().double_value
        self.num_points = self.declare_parameter('num_points',8).get_parameter_value().integer_value
        self.interactive_space = self.create_publisher(MarkerArray, 'interactive_space', 3)
        self.human_pose_pub = self.create_publisher(Odometry, 'human_global_pose', 10)
        self.human_robot_pub = self.create_publisher(Odometry, 'human_robot_pos', 10)
        self.human_state_pub = self.create_publisher(Int32, 'human_state', 10)
        
        self.x_vals, self.y_vals = self.generate_ellipse_points(self.semi_major_axis, self.semi_minor_axis, self.num_points)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.human_global = Odometry()
        self.human_robot = Odometry()
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_velocity = 0.0
        
        self.human_robot.header.frame_id="base_link"
        self.human_robot.child_frame_id="human_link"

        self.human_global.header.frame_id="odom"
        self.human_global.child_frame_id="human_link"
        self.raw_output = 0.0
        self.rc_sub = self.create_subscription(ScoutRCState,'/scout_rc_state', self.rc_callback,5)
        self.init_mode = 0

        self.image_width = self.declare_parameter('image_width', 1920).value
        self.image_center_x = self.image_width / 2.0

        self.last_command = "follow" 
        self.current_swd = 0

        self.encoder_subscriber = self.create_subscription(
            MotorState,
            'gimbal_state',
            self.encoder_callback,
            10
        )
        self.encoder_value = 0.0  

        self.genbot_sub = self.create_subscription(
            String,
            '/genbot',
            self.genbot_callback,
            10
        )

    def id_cb(self, msg):
        if self.locked_id != msg.data:
            self.locked_id = msg.data
            if self.locked_id == -1:
                self.get_logger().info("Received unlock command (-1), clearing target.")
                self.human = None
                self.detect_human = False
            else:
                self.get_logger().info(f"Target changed to ID: {self.locked_id}")
    
    def rc_callback(self, msg):
        self.current_swd = msg.swd  
        if self.current_swd == 0:
            self.init_mode = 0
        elif self.current_swd == 2:
            command = self.last_command 
            if command == "transport":
                self.init_mode = 0          
            elif command == "follow":
                self.init_mode = 2            
            elif command == "stop":
                self.init_mode = 0
        else:
            self.init_mode = msg.swd
                
    def genbot_callback(self, msg):
        valid_commands = ["transport", "follow", "stop"]
        if msg.data in valid_commands:
            self.last_command = msg.data

    def encoder_callback(self, msg):
        self.encoder_value = msg.angle_deg

    def _interactive_space(self):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(zip(self.x_vals, self.y_vals)):
            marker = Marker()
            marker.header.frame_id = "human_link"
            marker.ns = "ellipse"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.interactive_space.publish(marker_array)

    def _publish_cylinder_marker(self, pose: Pose):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.ns = 'human_cylinder'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = 0.01  
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.marker_radius * 2.0
        marker.scale.y = self.marker_radius * 2.0
        marker.scale.z = self.marker_height

        marker.color.r = 0.1
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.9

        self.marker_pub.publish(marker)
    
    def generate_ellipse_points(self, a, b, n):
        theta = np.linspace(0, 2 * np.pi, n)
        return a * np.cos(theta), b * np.sin(theta)

    def euler_to_quaternion(self,roll, pitch, yaw):
        return R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

    def skeletons_callback(self, msg):
        closest_human = None
        action_state = Int32()
        max_tracking_distance = 5.0  

        # ==========================================
        # 由於改為被動訂閱，這裡只需尋找畫面中 ID 相符的骨架
        # ==========================================
        if self.locked_id != -1 and self.locked_id is not None:
            found_target = False
            for obj in msg.objects:
                if not obj.tracking_available:
                    continue
                
                # 如果骨架 ID 符合外部指定的 locked_id
                if obj.label_id == self.locked_id:
                    distance = np.linalg.norm(obj.position[0:2])
                    if distance < max_tracking_distance:
                        closest_human = obj
                        found_target = True
                        break

            if found_target and closest_human is not None:
                self.human = closest_human
                self.action_state = closest_human.action_state
                action_state.data = self.action_state 
                self.human_state_pub.publish(action_state)
                self.detect_human = True
            else:
                # 找不到目標，單純將偵測狀態設為 False，不解除 locked_id
                self.detect_human = False
        else:
            self.detect_human = False

    def publish_human_tf(self):
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
        corners = human_obj.bounding_box_2d.corners
        x_sum = sum([c.kp[0] for c in corners])
        return x_sum / 4.0

    def timer_callback(self):
        if self.init_mode == 0 :
            Kp = 1.3
            Kd = 0.001
            setpoint = 0.0 
            
            error = setpoint - self.encoder_value
            error = (error + math.pi) % (2 * math.pi) - math.pi
            if not hasattr(self, 'last_error'):
                self.last_error = 0.0
            derivative = error - self.last_error
            self.last_error = error
            
            control = Kp * error + Kd * derivative
            
            gimbal_control = MotorCmd()
            gimbal_control.voltage = control

            self.gimbal_pub.publish(gimbal_control)
            self.detect_human = False
            return
            
        elif self.init_mode == 2:
            if self.detect_human and self.human is not None:
                target_x = self.get_2d_center_x(self.human)
                pixel_error = self.image_center_x - target_x
                normalized_error = pixel_error / self.image_center_x
                
                self.raw_output = normalized_error
                self.filtered_output = self.alpha * self.raw_output + (1 - self.alpha) * self.filtered_output
                control = float(3.5*self.filtered_output)
                
                gimbal_control = MotorCmd()
                gimbal_control.voltage = control
                
                self.linear_x = float(self.human.velocity[0])
                self.linear_y = float(self.human.velocity[1])
                self.angular_velocity = float(self.human.velocity[2])
                
                self.gimbal_pub.publish(gimbal_control)
                self.publish_human_tf()
                
            elif self.detect_human == False and self.human is not None:
                # 目標遺失，馬達電壓輸出為 0.0 讓其停止
                gimbal_control = MotorCmd()
                control = 0.0
                gimbal_control.voltage = control
                self.gimbal_pub.publish(gimbal_control)
                self.publish_human_tf()
                
            else:
                # 沒有目標，馬達電壓輸出為 0.0
                gimbal_control = MotorCmd()
                gimbal_control.voltage = 0.0
                self.gimbal_pub.publish(gimbal_control)
                self.publish_human_tf()

        self._interactive_space()

def main(args=None):
    rclpy.init(args=args)
    try:
        gimbal_tracking = GimbalTracking()
        rclpy.spin(gimbal_tracking)
    except KeyboardInterrupt:
        print("Shutting down node")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()