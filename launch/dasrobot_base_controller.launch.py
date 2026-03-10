#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():

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

    # Параметры для конвертации комманд управления
    wheel_base_x = LaunchConfiguration('wheel_base_x', default='0.3')
    wheel_base_y = LaunchConfiguration('wheel_base_y', default='0.2')
    max_wheel_rpm = LaunchConfiguration('max_wheel_rpm', default='200')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev', default='988')
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0325')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')


    wheel_base_x_declare = DeclareLaunchArgument(
            'wheel_base_x',
            default_value='0.3',
            description='The distance between the wheels on the x axis'
    )

    wheel_base_y_declare = DeclareLaunchArgument(
            'wheel_base_y',
            default_value='0.2',
            description='The distance between the wheels on the y axis'
    )

    max_wheel_rpm_declare = DeclareLaunchArgument(
            'max_wheel_rpm',
            default_value='200',
            description='Max limit RPM of wheel'
    )

    ticks_per_rev_declare = DeclareLaunchArgument(
            'ticks_per_rev',
            default_value='988',
            description='Count tick for one revolution period'
    )

    wheel_radius_declare = DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.0325',
            description='Wheel radius'
    )

    cmd_vel_topic_declare = DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Topic id for input command'
    )

    # Параметры для вычисления одометрии
    odom_topic_id = LaunchConfiguration('odom_topic_id', default='/odom')
    odom_frame_name = LaunchConfiguration('odom_frame_name', default='odom')
    base_frame_name = LaunchConfiguration('base_frame_name', default='base_footprint')
    rotation_scale = LaunchConfiguration('rotation_scale', default='0.45')

    odom_topic_id_declare = DeclareLaunchArgument(
        'odom_topic_id',
        default_value='/odom',
        description='Odom topic id'
    )

    odom_frame_name_declare = DeclareLaunchArgument(
        'odom_frame_name',
        default_value='odom',
        description='Odom frame name'
    )

    base_frame_name_declare = DeclareLaunchArgument(
        'base_frame_name',
        default_value='base_footprint',
        description='Base frame name'
    )

    rotation_scale_declare = DeclareLaunchArgument(
        'rotation_scale',
        default_value='0.45',
        description='Scale coeff for rotation scale'
    )
    
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
            'wheel_base_x': wheel_base_x,
            'wheel_base_y': wheel_base_y,
            'max_wheel_rpm': max_wheel_rpm,
            'wheel_radius': wheel_radius,
            'ticks_per_rev': ticks_per_rev,
            'cmd_vel_topic': cmd_vel_topic,
            'wheels_command_topic': serial_command_topic
        }],
    )
    
    odom_node = Node(
        package='dasrobot_base_controller',
        executable='dasrobot_odom',
        name='dasrobot_odom',
        output='screen',
        parameters=[{
            'wheel_base_x': wheel_base_x,
            'wheel_base_y': wheel_base_y,
            'wheel_radius': wheel_radius,
            'ticks_per_rev': ticks_per_rev,
            'wheel_encoders_topic': serial_response_topic,

            'odom_topic': odom_topic_id,
            'odom_frame': odom_frame_name,
            'base_frame': base_frame_name,
            'rotation_scale': rotation_scale,
        }]
    )
      
    ld = LaunchDescription()

    ld.add_action(serial_command_topic_declare)
    ld.add_action(serial_response_topic_declare)
    ld.add_action(serial_port_id_declare)
    ld.add_action(serial_port_baudrate_declare)
    ld.add_action(serial_port_polling_hz_declare)

    ld.add_action(wheel_base_x_declare)
    ld.add_action(wheel_base_y_declare)
    ld.add_action(max_wheel_rpm_declare)
    ld.add_action(ticks_per_rev_declare)
    ld.add_action(wheel_radius_declare)
    ld.add_action(cmd_vel_topic_declare)

    ld.add_action(odom_topic_id_declare)
    ld.add_action(odom_frame_name_declare)
    ld.add_action(base_frame_name_declare)
    ld.add_action(rotation_scale_declare)

    ld.add_action(serial_port_command_node)
    ld.add_action(cmd_converter_node)
    ld.add_action(odom_node)
    
    return ld
