import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    metric_arg = DeclareLaunchArgument(
        'metric',
        default_value='zvar',
        description='Evaluation metric: "zvar" (variance / estimated slope) or "zdiff" (max - min height)'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.1',
        description='Grid resolution in meters per cell'
    )

    return LaunchDescription([
        metric_arg,
        resolution_arg,
        launch_ros.actions.Node(
            package='floor_map',
            executable='floor_map_bucket',
            name='floor_map_bucket',
            parameters=[
                {'resolution': LaunchConfiguration('resolution')},
                {'min_x': -10.0},
                {'max_x': 10.0},
                {'min_y': -10.0},
                {'max_y': 10.0},
                {'metric': LaunchConfiguration('metric')},
                {'max_z_sigma': 0.02},
                {'max_z_diff': 0.08},
                {'max_scale_angle': 0.5236},  # 30 deg en radians
                {'max_scale_height': 0.5},   # 0.5 m
                {'min_points_bucket': 5},
                {'target_frame': 'world'},
            ],
            output='screen',
        ),
    ])
