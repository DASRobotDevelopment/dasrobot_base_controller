#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class DASRobotCmdConverterNode(Node):
    def __init__(self):
        super().__init__('dasrobot_cmd_converter_node')
        
        # Параметры
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheels_command_topic', '/serial_command')
        self.declare_parameter('max_wheel_rpm', 200)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('ticks_per_rev', 988)
        self.declare_parameter('wheel_base_x', 0.3)
        self.declare_parameter('wheel_base_y', 0.2)
        
        # Получаем параметры
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.wheels_command_topic = self.get_parameter('wheels_command_topic').value
        self.max_wheel_rpm = self.get_parameter('max_wheel_rpm').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.wheel_base_x = self.get_parameter('wheel_base_x').value
        self.wheel_base_y = self.get_parameter('wheel_base_y').value
        
        # ✅ ДИНАМИЧЕСКИЙ РАСЧЕТ rpm_per_mps!
        #self.rpm_per_mps = self.max_wheel_rpm / (2 * math.pi * self.wheel_radius * 60)
        self.rpm_per_mps = 60.0 / (2 * math.pi * self.wheel_radius)
        self.robot_radius = math.sqrt((self.wheel_base_x/2)**2 + (self.wheel_base_y/2)**2)
        
        self.wheels_pub = self.create_publisher(String, self.wheels_command_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        
        self.get_logger().info(f'{self.cmd_vel_topic} → {self.wheels_command_topic}')
        self.get_logger().info(f'wheel_radius={self.wheel_radius}m, max_rpm={self.max_wheel_rpm}')
        self.get_logger().info(f'rpm_per_mps={self.rpm_per_mps:.1f} RPM/(m/s)')

    def cmd_vel_callback(self, msg):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        
        wheel_rpm = self.twist_to_wheel_rpm(vx, vy, wz)
        formatted = self.format_wheel_rpm(wheel_rpm)
        cmd_string = f"v,{formatted[0]},{formatted[1]},{formatted[2]},{formatted[3]}"
        
        cmd_msg = String()
        cmd_msg.data = cmd_string
        self.wheels_pub.publish(cmd_msg)

    def twist_to_wheel_rpm(self, vx, vy, wz):
        """Twist → RPM с динамическим коэффициентом"""
        L, W = self.wheel_base_x, self.wheel_base_y
        
        # Mecanum kinematics
        speeds = [
            vx - vy - wz * (L + W)/2,  # FL
            vx + vy + wz * (L + W)/2,  # FR
            vx + vy - wz * (L + W)/2,  # RL
            vx - vy + wz * (L + W)/2   # RR
        ]
        
        # ✅ ДИНАМИЧЕСКАЯ конвертация m/s → RPM
        wheel_rpm = [int(s * self.rpm_per_mps) for s in speeds]
        
        # Ограничение
        return [max(min(rpm, self.max_wheel_rpm), -self.max_wheel_rpm) 
                for rpm in wheel_rpm]

    def format_wheel_rpm(self, rpm_values):
        return [f"{'+' if r>=0 else '-'}{abs(r):03d}" for r in rpm_values]

def main(args=None):
    rclpy.init(args=args)
    node = DASRobotCmdConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
