"""Treasure detector (objective 1).

ros2 launch treasure_detector treasure_detector.launch.py \
    [eval:=true] [calib_csv:=/path/calib.csv]

eval:=true compares detections with the ground truth TF frames of the scene.
calib_csv also logs (true distance, reading) pairs for fit_sensor_model.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('eval', default_value='true'),
        DeclareLaunchArgument('calib_csv', default_value=''),
        Node(
            package='treasure_detector', executable='treasure_detector',
            name='treasure_detector', output='screen',
            parameters=[{
                'world_frame': 'world',
                'tool_frame': 'VSV/Tool',
                # inverse sensor model (fit_sensor_model to recalibrate)
                'model.sigma': 2.0,
                'model.amplitude': 1.0,
                'model.noise_width': 0.1,
                'model.d_cap': 5.0,
                # map of the dry lake
                'grid.x_min': -60.0, 'grid.x_max': 60.0,
                'grid.y_min': -60.0, 'grid.y_max': 60.0,
                'grid.resolution': 0.05,
                # declare a treasure when the fused evidence is strong AND the
                # detector actually produced a high peak nearby
                'llr_threshold': 30.0,
                'peak_signal_threshold': 0.6,
                'min_separation': 6.0,
            }]),
        Node(
            package='treasure_detector', executable='treasure_eval',
            name='treasure_eval', output='screen',
            condition=IfCondition(LaunchConfiguration('eval')),
            parameters=[{'csv_file': LaunchConfiguration('calib_csv')}]),
    ])
