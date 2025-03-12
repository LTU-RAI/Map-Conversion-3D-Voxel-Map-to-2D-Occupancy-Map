from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapconversion',
            executable='map_conversion_oct_node',
            name='map_conversion',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'minimum_z': 1.0,
                'max_slope_ugv': 0.2,
                'slope_estimation_size': 2,
                'minimum_occupancy': 10,
                'partial_map_updates': True,
                #QoS parameters
                'subscriber_qos_reliable': True,
                'subscriber_qos_transient_local': False,
                'publisher_qos_reliable': True,
                'publisher_qos_transient_local': False,
            }],
            remappings=[
                ('octomap', 'octomap_binary')
            ]
        )
    ])
