#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # Параметры робота
    # cmd_vel_topic
    # wheels_commands_topic
    # max_wheel_rpm
    # wheel_radius
    # ticks_per_rev
    # wheel_base_x
    # wheel_base_y

    # wheel_encoders_topic
    # odom_topic
    # odom_frame
    # base_frame

    # Параметры последовательного порта
    serial_command_topic = LaunchConfiguration('serial_port_command_topic', default='/serial_command')
    serial_response_topic = LaunchConfiguration('serial_port_response_topic', default='/serial_response')

    serial_port_id = LaunchConfiguration('serial_port_id', default='/dev/ttyUSB0')
    serial_port_baudrate = LaunchConfiguration('serial_port_baudrate', default='115200')
    serial_port_polling_hz = LaunchConfiguration('serial_port_polling_hz', default='50')

    serial_command_topic_declare = DeclareLaunchArgument(
            'serial_command_topic',
            default_value='/serial_command',
            description='Topic name for serial command'
    )

    serial_response_topic_declare = DeclareLaunchArgument(
            'serial_response_topic',
            default_value='/serial_response',
            description='Topic name for serial response'
    )

    serial_port_id_declare = DeclareLaunchArgument(
            'serial_port_id',
            default_value='/dev/ttyUSB0',
            description='Serial port id'
    )

    serial_port_baudrate_declare = DeclareLaunchArgument(
            'serial_port_baudrate',
            default_value='115200',
            description='Serial port baudrate'
    )

    serial_port_polling_hz_declare = DeclareLaunchArgument(
            'serial_port_polling_hz',
            default_value='50',
            description='Serial port polling Hz'
    )

    # Путь к конфигурации
    # pkg_share = get_package_share_directory('dasrobot_base_controller')
    # urdf_file = os.path.join(pkg_share, 'urdf', 'dasrobot.urdf')
    # config_dir = os.path.join(pkg_share, 'config')
    
    serial_port_command_node = Node(
        package='dasrobot_base_controller',
        executable='dasrobot_serial',
        name='dasrobot_serial',
        output='screen',
        parameters=[{
            'baudrate': serial_port_baudrate,
            'serial_port_id': serial_port_id,
            'serial_port_polling_hz': serial_port_polling_hz,
            'command_topic': serial_command_topic,
            'response_topic': serial_response_topic
        }],
    )
    
    cmd_converter_node = Node(
        package='dasrobot_base_controller',
        executable='dasrobot_cmd_converter',
        name='dasrobot_cmd_converter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_base_x': 0.3,
            'wheel_base_y': 0.2,
            'max_wheel_rpm': 100,
            'wheel_radius': 0.0325,
            'ticks_per_rev': 988
        }],
    )
    
    # dasrobot_odom (одометрия + /joint_states)
    # odom_node = Node(
    #     package='dasrobot_base_controller',
    #     executable='dasrobot_odom',
    #     name='dasrobot_odom',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'wheel_radius': 0.0325,
    #         'ticks_per_rev': 988,
    #         'wheel_encoders_topic': '/serial_response',
    #         'wheel_base_x': 0.3,
    #         'wheel_base_y': 0.2,
    #         'rotation_scale': 0.5,  # Калибровка поворота
    #         'linear_scale': 1.0
    #     }]
    # )
    
    # RViz2 (визуализация)
    # rviz_config = os.path.join(pkg_share, 'rviz', 'dasrobot.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    
    ld = LaunchDescription()

    ld.add_action(serial_command_topic_declare)
    ld.add_action(serial_response_topic_declare)
    ld.add_action(serial_port_id_declare)
    ld.add_action(serial_port_baudrate_declare)
    ld.add_action(serial_port_polling_hz_declare)

    ld.add_action(serial_port_command_node)
    
    return ld
