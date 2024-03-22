import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='reactive_gap_follow',
            executable='reactive_gap_follow_node',
            name='reactive_gap_follow_node',
            output='screen',
            parameters=[{'bubble_radius': 0.0},
                        {'smoothing_filter_size': 0.0},
                        {'truncated_coverage_angle': 0.0},
                        {'velocity': 0.0},
                        {'max_accepted_distance': 0.0},
                        {'error_based_velocities': {'high': 0.0, 'medium': 0.0, 'low': 0.0}},
                        {'steering_angle_reactivity': 0.0}]
        )
    ])

