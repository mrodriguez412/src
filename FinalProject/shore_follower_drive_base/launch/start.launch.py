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
            default_value='/home/GTL/rmartine/Documents/gitcommun/src/FinalProject/Recorded_data_shore/output/models/model_ce_ep23_val74.pt',
            description='pytorch checkpoint to load'),
        launch_ros.actions.Node(
            package='shore_follower_drive_base', executable='shore_follower_drive_node', name='shore_follower_drive',
            parameters=[
                {'~/model': model},
                {'~/regression': False},
                # vsv_driver clamps the turning radius to 5 m (angular.z <= linear.x / 5),
                # so twist_factor must stay <= linear_vel / 5 or the turn is cut.
                # 1.0 m/s is also the lowest speed at which a recorded turn (|w| > 0.2) is possible.
                {'~/linear_vel': 1.0},
                {'~/twist_factor': 0.2},  # full steering lock at 1.0 m/s
                {'~/dropout': 0.5},       # only used at train time, kept equal to --dropout for clarity
                ],
            remappings=[
                ('~/image', '/vrep/vision/image'),
                ('~/twist', '/base_mux/autoCommand'),
                ],
            output='screen'),

        # Joystick + muxes: button 4 = manual driving, button 5 = network (auto) driving
        # (0/1/2 are taken by the arm deploy/park/move buttons in vsv_teleop)
        IncludeLaunchDescription(
              PythonLaunchDescriptionSource([get_package_share_directory('vsv_teleop'),
                        '/launch/vsv_teleop.launch.py']),
              launch_arguments={'base_joy_button': '4', 'base_auto_button': '5'}.items(),
              )
    ])
