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
                #('pathIn', '/path'),
                #('pathOut', '/path_3d')
            ],
            parameters=[{
                'use_collision_sphere': False,
                'collision_radius': 1.0,
                'path_offset': 0.0},
                'path_smothing_length': 5
                #QoS parameters
                'subscriber_qos_reliable': True,
                'subscriber_qos_transient_local': False,
                'publisher_qos_reliable': True,
                'publisher_qos_transient_local': False,
            }]
        )
    ])
