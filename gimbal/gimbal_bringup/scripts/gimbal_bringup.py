#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
import struct
import math
from gimbal_msg.msg import MotorState
from gimbal_msg.msg import MotorCmd
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class GM6020Reader205(Node):
    def __init__(self):
        super().__init__("gimbal_bringup")

        can_interface = "can3"
        self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')

        # Declare camera position parameters (fixed x, y, z)
        self.declare_parameter('camera_x', 0.0)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.0)
        self.camera_x = self.get_parameter('camera_x').get_parameter_value().double_value
        self.camera_y = self.get_parameter('camera_y').get_parameter_value().double_value
        self.camera_z = self.get_parameter('camera_z').get_parameter_value().double_value


        # Publisher kiểu mới
        self.pub_state = self.create_publisher(MotorState, "gimbal_state", 3)

        # Subscriber lệnh điều khiển motor
        self.sub_cmd = self.create_subscription(
            MotorCmd,
            "gimbal_control",
            self.cmd_callback,
            10
        )

        self.get_logger().info(f"GM6020 reader (0x205) on {can_interface}")
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.timer = self.create_timer(0.001, self.read_can)


    def euler_to_quaternion(self,roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        return R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    

    def cmd_callback(self, cmd: MotorCmd):
        """Nhận lệnh điều khiển từ topic gimbal_control và gửi CAN theo mode"""
        if cmd.mode == 0:  # voltage mode
            # Giới hạn voltage trong ±7V
            voltage_limited = max(min(cmd.voltage, 5.0), -5.0)
            # Chuyển voltage float → raw (-25000~25000)
            voltage_raw = int(voltage_limited / 12.0 * 25000)

            # Byte high/low
            high_byte = (voltage_raw >> 8) & 0xFF
            low_byte = voltage_raw & 0xFF

            # Data 8 byte
            data = bytes([high_byte, low_byte] + [0]*6)

            msg = can.Message(arbitration_id=0x1FF, data=data, is_extended_id=False)
            try:
                self.bus.send(msg)
                self.get_logger().debug(f"Voltage mode: {voltage_limited:.2f} V → raw={voltage_raw}")
            except can.CanError as e:
                self.get_logger().error(f"CAN send failed: {e}")

        elif cmd.mode == 1:  # current mode
            # Giới hạn current trong ±3A
            current_limited = max(min(cmd.current_amp, 2.0), -2.0)
            # Chuyển current float → raw (-16384~16384)
            current_raw = int(current_limited / 3.0 * 16384)

            # Byte high/low
            high_byte = (current_raw >> 8) & 0xFF
            low_byte = current_raw & 0xFF

            # Data 8 byte
            data = bytes([high_byte, low_byte] + [0]*6)

            msg = can.Message(arbitration_id=0x1FE, data=data, is_extended_id=False)
            try:
                self.bus.send(msg)
                self.get_logger().debug(f"Current mode: {current_limited:.3f} A → raw={current_raw}")
            except can.CanError as e:
                self.get_logger().error(f"CAN send failed: {e}")

    def parse_gm6020(self, data):
        # angle (0–8191) → radian
        # Công thức: angle_raw * (2 * pi / 8192)
        angle_raw = (data[0] << 8) | data[1]
        angle_rad = angle_raw * (2.0 * math.pi) / 8192.0
        angle_rad = angle_rad - 5.269224166870117

        # speed: signed int16 (rpm) → Chuyển sang rad/s nếu cần
        # Ở đây tôi giữ nguyên rpm theo cấu trúc cũ của bạn, 
        # nhưng nếu muốn sang rad/s: speed_rpm * (2 * pi / 60)
        speed_rpm = struct.unpack(">h", data[2:4])[0]

        # current: signed int16 → Amp
        current_raw = struct.unpack(">h", data[4:6])[0]
        current_amp = float(current_raw) / 16384.0 * 3.0

        # temperature: uint8
        temperature = float(data[6])

        return angle_rad, float(speed_rpm), current_amp, temperature

    def read_can(self):
        msg = self.bus.recv(0.0001)
        if msg is None:
            return

        if msg.arbitration_id == 0x205:
            angle, speed, current_amp, temp = self.parse_gm6020(msg.data)

            state = MotorState()
            state.angle_deg = angle
            state.speed_rpm = speed
            state.current_amp = current_amp
            state.temperature_c = temp

            self.pub_state.publish(state)


             # Broadcast TF from base_link to camera
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'camera_gimbal'
            # Set fixed position
            t.transform.translation.x = self.camera_x
            t.transform.translation.y = self.camera_y
            t.transform.translation.z = self.camera_z
            # Convert encoder angle (degrees) to quaternion (rotation around z-axis for yaw)
            # print(encoder_angle)
            quaternion = self.euler_to_quaternion(0.0, 0.0, angle)
            # print(quaternion)
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]
            # Send the transform
            self.tf_broadcaster.sendTransform(t)
        
            # self.get_logger().info(
            #     f"[0x205] Angle={angle:.2f} deg | Speed={speed:.1f} rpm | "
            #     f"Current={current_amp:.3f} A | Temp={temp}°C"
            # )


def main(args=None):
    rclpy.init(args=args)
    node = GM6020Reader205()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

