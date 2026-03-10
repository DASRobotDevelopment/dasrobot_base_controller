#!/usr/bin/env python3
import time
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

class DASRobotSerialNode(Node):
    def __init__(self):
        super().__init__('dasrobot_serial_node')

        # Параметры
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_port_id', '/dev/ttyUSB0')
        self.declare_parameter('serial_port_polling_hz', 50)
        self.declare_parameter('command_topic', '/serial_commands')
        self.declare_parameter('response_topic', '/serial_response')

        self.baudrate = self.get_parameter('baudrate').value
        self.serial_port_id = self.get_parameter('serial_port_id').value
        self.serial_port_polling_hz = self.get_parameter('serial_port_polling_hz').value
        self.command_topic = self.get_parameter('command_topic').value
        self.response_topic = self.get_parameter('response_topic').value

        self.get_logger().info(f'Config: port={self.serial_port_id}, '
                               f'polling={self.serial_port_polling_hz}Hz, '
                               f'command={self.command_topic}, output={self.response_topic}')
        
        # Serial
        self.serial_port = None
        self.open_serial_port()

        # Подписка
        self.subscription = self.create_subscription(
            String, self.command_topic, self.command_callback, 10)
        self.get_logger().info(f'Subscribed to {self.command_topic}')

        # Издатель
        self.publisher = self.create_publisher(String, self.response_topic, 10)
        self.get_logger().info(f'Publishing to {self.response_topic}')

        # Таймер polling
        self.serial_polling_timer = self.create_timer(
            1.0/self.serial_port_polling_hz, self.serial_poll)
        self.get_logger().info(f'Serial polling at {self.serial_port_polling_hz} Hz')

    def open_serial_port(self):
        try:
            self.serial_port = serial.Serial(
                self.serial_port_id, 
                self.baudrate, 
                timeout=0.01,  # ✅ 10мс вместо 1с!
                bytesize=8,
                parity='N',
                stopbits=1)  # Быстрый timeout
            time.sleep(2)
            self.get_logger().info(f'Serial opened: {self.serial_port_id}')
        except Exception as e:
            self.get_logger().error(f'Serial open error: {e}')

    def command_callback(self, msg):
        if self.serial_port and self.serial_port.is_open:
            try:
                command = msg.data + '\n'  # Правильный \n
                self.serial_port.write(command.encode('utf-8'))
                self.get_logger().debug(f'Sent: {msg.data}')
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')
        else:
            self.get_logger().warn('Serial not open')

    def serial_poll(self):
        if self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting > 0:
            try:
                response = self.serial_port.readline().decode('utf-8', errors='ignore').rstrip()
                if response:  # Пропуск пустых
                    feedback_msg = String()
                    feedback_msg.data = response
                    self.publisher.publish(feedback_msg)
                    self.get_logger().debug(f'Received: {response}')
            except Exception as e:
                self.get_logger().warn(f'Poll error: {e}')

    def destroy_node(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial closed')
        super().destroy_node()  # Без return

def main(args=None):
    rclpy.init(args=args)
    node = DASRobotSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
