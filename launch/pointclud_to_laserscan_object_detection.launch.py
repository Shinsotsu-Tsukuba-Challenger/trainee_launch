from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/livox/lidar'),
                        ('scan', '/scan_obj_detect')],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.5,
                'max_height': 0.6,
                'angle_min': -3.1415,  # -M_PI [0.78625 3.1415]
                'angle_max': 3.1415,  # M_PI [0.78625 3.1415]
                'angle_increment': 0.0174,  # M_PI/180.0 [0.00435 0.0174]
                'scan_time': 0.3333,
                'range_min': 0.3,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_object_detection'
        )
    ])