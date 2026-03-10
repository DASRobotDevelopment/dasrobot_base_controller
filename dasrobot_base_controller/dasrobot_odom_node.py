#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class DASRobotOdomNode(Node):
    def __init__(self):
        super().__init__('dasrobot_odom_node')

        # Параметры
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('ticks_per_rev', 988)
        self.declare_parameter('wheel_encoders_topic', '/serial_response')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('wheel_base_x', 0.210)
        self.declare_parameter('wheel_base_y', 0.165)
        self.declare_parameter('rotation_scale', 0.45)

        # Получаем параметры
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.wheel_encoders_topic = self.get_parameter('wheel_encoders_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheel_base_x = float(self.get_parameter('wheel_base_x').value)
        self.wheel_base_y = float(self.get_parameter('wheel_base_y').value)
        self.rotation_scale = float(self.get_parameter('rotation_scale').value)
        
        # Имена joint'ов
        self.wheel_joints = [
            'base_front_left_wheel_joint',
            'base_front_right_wheel_joint', 
            'base_rear_left_wheel_joint',
            'base_rear_right_wheel_joint'
        ]

        # ✅ ИСПРАВЛЕННОЕ состояние
        self.current_ticks = [0, 0, 0, 0]
        self.last_ticks = None  # ← КЛЮЧЕВОЕ исправление!
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_time = None

        self.robot_radius = math.sqrt(
            (self.wheel_base_x / 2)**2 + 
            (self.wheel_base_y / 2)**2
        )

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.encoders_sub = self.create_subscription(
            String, self.wheel_encoders_topic, self.encoders_callback, 10)

        # 🔥 ТАЙМЕР 20Гц (50мс)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info(f'🔥 Timer 20Hz | Sub: {self.wheel_encoders_topic}')
        self.get_logger().info(f'Pub: /joint_states, {self.odom_topic}')

    def encoders_callback(self, msg):
        """ТОЛЬКО обновляет тики"""
        try:
            parts = msg.data.strip().split(',')
            if len(parts) != 5 or parts[0] != 'e':
                return
            self.current_ticks = [int(parts[i]) for i in range(1, 5)]
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')

    def timer_callback(self):
        """🔥 20Гц: одометрия + joint_states"""
        current_time = self.get_clock().now()
        
        # ✅ Инициализация (ждём первые тики)
        if self.last_ticks is None:
            if any(t > 0 for t in self.current_ticks):  # Есть данные!
                self.last_ticks = self.current_ticks[:]
                self.last_time = current_time
                self.get_logger().info(f'✅ CALIBRATED: {self.current_ticks}')
            return
        
        # ✅ Минимальный интервал
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt < 0.005:
            return
        
        delta_ticks = [float(self.current_ticks[i] - self.last_ticks[i]) for i in range(4)]
        
        # ✅ Обновляем одометрию
        self.update_odometry_from_ticks(self.current_ticks, current_time, delta_ticks, dt)
        
        # ✅ Joint states для RViz
        self.publish_wheel_joints(self.current_ticks)

    def update_odometry_from_ticks(self, current_ticks, current_time, delta_ticks, dt):
        """Расчёт и публикация одометрии"""
        # Дифференциальная одометрия (left/right)
        left_avg_ticks  = (delta_ticks[0] + delta_ticks[2]) / 2.0   # FL+RL
        right_avg_ticks = (delta_ticks[1] + delta_ticks[3]) / 2.0   # FR+RR
        
        wheel_circ = 2 * math.pi * self.wheel_radius
        left_dist  = left_avg_ticks  * wheel_circ / self.ticks_per_rev
        right_dist = right_avg_ticks * wheel_circ / self.ticks_per_rev
        
        # Поворот и движение
        dtheta = self.rotation_scale * (right_dist - left_dist) / self.wheel_base_x
        avg_dist = (left_dist + right_dist) / 2.0
        
        # Интеграция позиции
        half_th = self.th + dtheta / 2.0
        self.x += avg_dist * math.cos(half_th)
        self.y += avg_dist * math.sin(half_th)
        self.th += dtheta
        
        # Odometry сообщение
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position = Point(x=float(self.x), y=float(self.y), z=0.0)
        odom.pose.pose.orientation = self.yaw_to_quat(self.th)
        odom.twist.twist.linear.x = float(avg_dist / dt) if dt > 0 else 0.0
        odom.twist.twist.angular.z = float(dtheta / dt) if dt > 0 else 0.0
        self.odom_pub.publish(odom)
        
        self.publish_odom_tf(current_time)
        
        # ✅ Сохраняем для следующего цикла
        self.last_ticks = current_ticks[:]
        self.last_time = current_time

    def publish_wheel_joints(self, ticks):
        """JointState для вращения колёс в RViz"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.wheel_joints
        
        normalized_angles = []
        for tick in ticks:
            angle_rad = (float(tick) % self.ticks_per_rev) * 2 * math.pi / self.ticks_per_rev
            normalized_angles.append(angle_rad)
        
        joint_msg.position = normalized_angles
        self.joint_pub.publish(joint_msg)

    def yaw_to_quat(self, yaw):
        """Yaw → Quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def publish_odom_tf(self, current_time):
        """odom → base_footprint TF"""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation = self.yaw_to_quat(self.th)
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.get_logger().info('Shutting down odom node')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DASRobotOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
