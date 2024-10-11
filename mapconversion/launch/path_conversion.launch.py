from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapconversion',
            executable='path_converter_node',
            name='path_converter',
            output='screen',
            remappings=[
                ('pathIn', '/path'),
                ('pathOut', '/path_3d')
            ],
            parameters=[
                {'use_collision_sphere': True},
                {'collision_radius': 0.50},
                {'path_offset': 1},
                {'path_smothing_length': 20}
            ]
        )
    ])
