from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mapconversion',
            executable='map_conversion_oct_node',
            name='map_conversion',
            output='screen',
            parameters=[
                {'minimum_z': 1.0},
                {'minimum_occupancy': 10},
                {'map_frame': 'map'},
                {'max_slope_ugv': 0.2},
                {'slope_estimation_size': 2}
            ],
            remappings=[
                ('octomap', 'octomap_binary')
            ]
        )
    ])
