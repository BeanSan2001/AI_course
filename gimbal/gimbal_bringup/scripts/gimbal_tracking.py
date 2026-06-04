#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32,Int32,String
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
from geometry_msgs.msg import Twist, Pose,  Point, Quaternion
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
        self.human = None
        self.detect_human = False
        self.locked_id = None
        self.min_tracking = 2.0  # hoặc để là tham số ROS
        self.gimbal_pub = self.create_publisher(MotorCmd, 'gimbal_control', 10)
        self.filtered_output = 0.0  # giá trị lọc đầu ra khởi tạo
        self.alpha = 0.1  # hệ số lọc, bạn có thể tinh chỉnh
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_frame = "zed_gimbal_left_camera_frame"  # hoặc "zed_left_camera_frame", tùy theo cấu hình bạn dùng
        self.human_frame = "human_link"
        self.reacquire_timeout = Duration(seconds=15.0)
        self.marker_radius: float = self.declare_parameter('marker_radius', 0.5).value
        self.marker_height: float = self.declare_parameter('marker_height', 0.01).value
        self.marker_pub = self.create_publisher(Marker, '/human_marker', 10)
        self.semi_major_axis = self.declare_parameter('semi_major_axis', 1.1).get_parameter_value().double_value
        # self.semi_minor_axis = self.declare_parameter('semi_minor_axis', 0.9).get_parameter_value().double_value
        self.semi_minor_axis = self.declare_parameter('semi_minor_axis', 0.8).get_parameter_value().double_value
        self.num_points = self.declare_parameter('num_points',8).get_parameter_value().integer_value
        self.interactive_space = self.create_publisher(MarkerArray, 'interactive_space', 3)
        self.human_pose_pub = self.create_publisher(Odometry, 'human_global_pose', 10)
        self.human_robot_pub = self.create_publisher(Odometry, 'human_robot_pos', 10)
        self.human_state_pub = self.create_publisher(Int32, 'human_state', 10)
        self.locked_id_pub = self.create_publisher(Int32, 'locked_id', 10)
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
        self.encoder_value = 0.0  # Lưu giá trị encoder mới nhất

        self.genbot_sub = self.create_subscription(
            String,
            '/genbot',
            self.genbot_callback,
            10
        )

    
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
        else:
            pass

    def encoder_callback(self, msg):
        self.encoder_value = msg.angle_deg

    def _interactive_space(self):
        # Tạo MarkerArray từ ellipse quanh người
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(zip(self.x_vals, self.y_vals)):
            marker = Marker()
            marker.header.frame_id = "human_link"
            # marker.header.stamp = self.get_clock().now().to_msg()
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

        # Publish MarkerArray
        self.interactive_space.publish(marker_array)
    # —— Marker ——
    def _publish_cylinder_marker(self, pose: Pose):
        """Publish a CUBE marker around the human to represent a flat circle."""
        marker = Marker()
        marker.header.frame_id = "odom"
        # marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'human_cylinder'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = 0.01  # Flat on the ground
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
        """Generates n points on an ellipse with semi-major axis a and semi-minor axis b."""
        theta = np.linspace(0, 2 * np.pi, n)
        return a * np.cos(theta), b * np.sin(theta)

    def euler_to_quaternion(self,roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        return R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

    def skeletons_callback(self, msg):
        closest_human = None
        action_state = Int32()
        min_distance = float('inf')
        max_tracking_distance = 5.0  # Giới hạn khoảng cách để tracking
        reacquire_distance = 2.0     # Khoảng cách tối đa để tìm người mới nếu bị mất quá lâu

        # Get the current time for tracking lost timeout logic
        now = self.get_clock().now()
        

        # Nếu đã mất tracking quá 15s thì reset locked_id để tìm người mới
        if self.locked_id is not None and not self.detect_human:
            if self.last_lost_time is None:
                self.last_lost_time = now
            elif now - self.last_lost_time > self.reacquire_timeout:
                self.get_logger().info("Lost tracking for too long, re-acquiring new target")
                self.locked_id = None
                self.last_lost_time = None  # reset sau khi đã unlock
        else:
            self.last_lost_time = None  # nếu đang tracking, reset thời gian mất tracking

        # --- CHẾ ĐỘ CHƯA KHÓA NGƯỜI ---
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
                self.action_state = closest_human.action_state
                action_state.data = self.action_state 
                self.human_state_pub.publish(action_state)
                self.detect_human = True
                self.get_logger().info(f"Locked on person ID {self.locked_id} at distance {min_distance:.2f} m")
                # Publish locked_id 
                id_msg = Int32()
                id_msg.data = self.locked_id
                self.locked_id_pub.publish(id_msg)
            else:
                self.detect_human = False
                self.get_logger().info("No person within range to lock")

        # --- CHẾ ĐỘ ĐÃ KHÓA NGƯỜI ---
        else:
            locked_position = np.array(self.human.position[0:2])
            for obj in msg.objects:
                if not obj.tracking_available:
                    continue
                nerest_distance = np.linalg.norm(np.array(obj.position[0:2]) - locked_position)
                distance = np.linalg.norm(obj.position[0:2])
                if nerest_distance < min_distance and distance < max_tracking_distance:
                    min_distance = nerest_distance
                    closest_human = obj

            if closest_human is not None:
                self.human = closest_human
                self.action_state = closest_human.action_state
                action_state.data = self.action_state 
                self.human_state_pub.publish(action_state)
                self.detect_human = True
                # self.get_logger().info(f"Tracking locked person, now closest match at {min_distance:.2f} and distance {distance:.2f} m")
            else:
                self.detect_human = False
                self.get_logger().info("Lost tracking of locked person")

    def publish_human_tf(self):
        if self.human is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.human_frame

        # Position
        t.transform.translation.x = float(self.human.position[0])
        t.transform.translation.y = float(self.human.position[1])
        t.transform.translation.z = float(self.human.position[2])

        # Orientation
        quat = self.human.global_root_orientation  # [x, y, z, w]
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    def get_2d_center_x(self, human_obj):
        """計算 2D Bounding Box 的 X 軸中心點"""
        corners = human_obj.bounding_box_2d.corners
        x_sum = sum([c.kp[0] for c in corners])
        return x_sum / 4.0


    def timer_callback(self):
        if self.init_mode == 0 :
            Kp = 1.3
            Kd = 0.001
            setpoint = 0.0  # target angle (degree or rad, depends on encoder)
            # Calculate error and derivative
            error = setpoint - self.encoder_value
            error = (error + math.pi) % (2 * math.pi) - math.pi
            if not hasattr(self, 'last_error'):
                self.last_error = 0.0
            derivative = error - self.last_error
            self.last_error = error
            # PD control output
            control = Kp * error + Kd * derivative
            # Publish control to gimbal

            gimbal_control = MotorCmd()
            gimbal_control.voltage = control

            self.gimbal_pub.publish(gimbal_control)
            # self.publish_human_tf()
            self.detect_human = False
            self.locked_id = None
            return
        elif self.init_mode == 2:
            if self.detect_human and self.human is not None:
                # self.raw_output = self.human.position[1]
                # if abs(raw_output)<0.1:
                #     control = 0.0
                # else:
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
                # Publish TF
                self.publish_human_tf()
            elif self.detect_human == False and self.human is not None:
                # print(self.human)
                gimbal_control = MotorCmd()
                # control = float(3.0 * np.sign(self.human.position[1]))
                control = float(1.5)
                gimbal_control.voltage = control
                self.gimbal_pub.publish(gimbal_control)
                self.publish_human_tf()
            else:
                # print(self.human)
                gimbal_control = MotorCmd()
                gimbal_control.voltage = 0.0
                self.gimbal_pub.publish(gimbal_control)
                self.publish_human_tf()

        # if self.human is not None:
        #     try:
        #         human_global_tf = self.tf_buffer.lookup_transform('odom', 'human_link', rclpy.time.Time())

        #         self.human_global.header.stamp = self.get_clock().now().to_msg()

        #         self.human_global.pose.pose.position.x = human_global_tf.transform.translation.x
        #         self.human_global.pose.pose.position.y = human_global_tf.transform.translation.y
        #         self.human_global.pose.pose.position.z = human_global_tf.transform.translation.z

        #         self.human_global.pose.pose.orientation.x = human_global_tf.transform.rotation.x
        #         self.human_global.pose.pose.orientation.y = human_global_tf.transform.rotation.y
        #         self.human_global.pose.pose.orientation.z = human_global_tf.transform.rotation.z
        #         self.human_global.pose.pose.orientation.w = human_global_tf.transform.rotation.w

        #         self.human_global.twist.twist.linear.x =  self.linear_x 
        #         self.human_global.twist.twist.linear.y =  self.linear_y
        #         self.human_global.twist.twist.angular.z = self.angular_velocity

        #         self.human_pose_pub.publish(self.human_global)


        #         human_robot_tf = self.tf_buffer.lookup_transform('base_link', 'human_link', rclpy.time.Time())


        #         self.human_robot.header.stamp = self.get_clock().now().to_msg()

        #         self.human_robot.pose.pose.position.x = human_robot_tf.transform.translation.x
        #         self.human_robot.pose.pose.position.y = human_robot_tf.transform.translation.y
        #         self.human_robot.pose.pose.position.z = human_robot_tf.transform.translation.z

        #         self.human_robot.pose.pose.orientation.x = human_robot_tf.transform.rotation.x
        #         self.human_robot.pose.pose.orientation.y = human_robot_tf.transform.rotation.y
        #         self.human_robot.pose.pose.orientation.z = human_robot_tf.transform.rotation.z
        #         self.human_robot.pose.pose.orientation.w = human_robot_tf.transform.rotation.w

        #         self.human_robot.twist.twist.linear.x =  self.linear_x 
        #         self.human_robot.twist.twist.linear.y =  self.linear_y
        #         self.human_robot.twist.twist.angular.z = self.angular_velocity

        #         self.human_robot_pub.publish(self.human_robot)
                
        #     except TransformException as ex:
        #         self.get_logger().info(f'Could not transform: {ex}')
        #         return

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