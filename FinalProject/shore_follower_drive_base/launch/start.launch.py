from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    model = LaunchConfiguration('model')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='/home/GTL/mrodrgue/ros2_ws/src/Project2/Recorded_data_shore/output/models/model_ce_ep19_val70.pt',
            description='pytorch checkpoint to load'),
        launch_ros.actions.Node(
            package='shore_follower_drive_base', executable='shore_follower_drive_node', name='shore_follower_drive',
            parameters=[
                {'~/model': model},
                {'~/regression': False},
                {'~/linear_vel': 0.3},    # same as the joystick max forward speed (scale_linear_x)
                {'~/twist_factor': 0.4},  # rotation when sure to turn, between threshold (0.2) and joystick max (0.5)
                {'~/dropout': 0.5},       # only used at train time, kept equal to --dropout for clarity
                ],
            remappings=[
                ('~/image', '/vrep/kision/image'),
                ('~/twist', '/mux/autoCommand'),
                ],
            output='screen'),

        # Joystick + mux: button 0 = manual control, button 1 = network (auto) control
        IncludeLaunchDescription(
              PythonLaunchDescriptionSource([get_package_share_directory('vrep_ros_teleop'),
                        '/teleop_mux.launch.py'])
              )
    ])
