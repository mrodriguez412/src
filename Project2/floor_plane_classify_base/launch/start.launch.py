from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    model = LaunchConfiguration('model')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='/home/GTL/mrodrgue/ros2_ws/src/Project2/Recorded_data/output/models/model_ce_94.pt',
            description='pytorch checkpoint to load'),
        launch_ros.actions.Node(
            package='floor_plane_classify_base', executable='floor_plane_classify_node', name='floor_plane_classify',
            parameters=[
                {'~/model': model},
                {'~/thumb_size': 32},
                {'~/num_classes': 2},   # must match --classes used in train_floor_classifier.sh
                {'~/dropout': 0.2},     # must match --dropout used for training
                ],
            remappings=[
                ('~/image', '/vrep/kision/image'),
                ],
            output='screen')
    ])
