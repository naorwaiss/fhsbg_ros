import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fhsbg',
            executable='dist_with_ros',
            name='dist_with_ros_node',
            #output='screen',
        ),
        Node(
            package='fhsbg',
            executable='aruco_filter',
            name='camera_modification_node',
            # output='screen',
        ),
        Node(
            package='fhsbg',
            executable='camera_modification',
            name='camera_modification_node',
            #output='screen',
        ),


    ])
