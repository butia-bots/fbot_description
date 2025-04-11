import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # base_footprint to camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_footprint',
            arguments=['--x', '0.067050', 
                       '--y', '0.000000', 
                       '--z', '-0.004250',
                       '--qx', '0.000000',
                       '--qy', '0.000000',
                       '--qz', '0.000000',
                       '--qw', '1.000000',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'camera_link'],
        ),
        
        # base_footprint to left_wheel_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='left_wheel_to_base_footprint',
            arguments=['--x', '0.000000', 
                       '--y', '0.217500', 
                       '--z', '-0.110000',
                       '--qx', '0.000000',
                       '--qy', '0.000000',
                       '--qz', '0.000000',
                       '--qw', '1.000000',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'left_wheel_link'],
        ),
        
        # base_footprint to head_tilt_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='head_tilt_to_base_footprint',
            arguments=['--x', '0.191000', 
                       '--y', '0.000000', 
                       '--z', '0.868000',
                       '--qx', '0.000000',
                       '--qy', '0.000000',
                       '--qz', '0.000000',
                       '--qw', '1.000000',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'head_tilt_link'],
        ),
        
        # base_footprint to right_wheel_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='right_wheel_to_base_footprint',
            arguments=['--x', '0.000000', 
                       '--y', '-0.217500', 
                       '--z', '-0.110000',
                       '--qx', '0.000000',
                       '--qy', '0.000000',
                       '--qz', '0.000000',
                       '--qw', '1.000000',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'right_wheel_link'],
        ),
        
        # base_footprint to head_pan_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='head_pan_to_base_footprint',
            arguments=['--x', '0.191000', 
                       '--y', '0.000000', 
                       '--z', '0.818000',
                       '--qx', '0.000000',
                       '--qy', '0.000000',
                       '--qz', '0.000000',
                       '--qw', '1.000000',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'head_pan_link'],
        ),
    ])
