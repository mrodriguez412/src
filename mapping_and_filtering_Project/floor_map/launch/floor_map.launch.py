import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    method_arg = DeclareLaunchArgument(
        'method',
        default_value='bucket',
        description='Mapping method to run: "bucket" or "sobel"'
    )

    metric_arg = DeclareLaunchArgument(
        'metric',
        default_value='zvar',
        description='Evaluation metric for bucket method: "zvar" (variance) or "zdiff" (height difference)'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.1',
        description='Grid resolution in meters per cell'
    )

    # Nœud 1 : Méthode par seaux / hachage spatial
    bucket_node = launch_ros.actions.Node(
        package='floor_map',
        executable='floor_map_bucket',
        name='floor_map_bucket',
        condition=LaunchConfigurationEquals('method', 'bucket'),
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
    )

    # Nœud 2 : Méthode par gradient 2D de Sobel sur DEM
    sobel_node = launch_ros.actions.Node(
        package='floor_map',
        executable='floor_map_sobel',
        name='floor_map_sobel',
        condition=LaunchConfigurationEquals('method', 'sobel'),
        parameters=[
            {'resolution': LaunchConfiguration('resolution')},
            {'min_x': -10.0},
            {'max_x': 10.0},
            {'min_y': -10.0},
            {'max_y': 10.0},
            {'max_slope_angle': 0.2618},  # 15 deg en radians
            {'max_scale_angle': 0.5236},  # 30 deg en radians
            {'max_z_diff': 0.08},         # Seuil marches internes
            {'min_points_bucket': 5},
            {'target_frame': 'world'},
        ],
        output='screen',
    )

    return LaunchDescription([
        method_arg,
        metric_arg,
        resolution_arg,
        bucket_node,
        sobel_node,
    ])
