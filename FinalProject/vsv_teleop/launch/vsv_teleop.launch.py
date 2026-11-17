"""Same robot stack as vrep_vsv_driver/vsv_geom_mux.launch.py, with:
  * car-like steering (car_teleop) instead of the angular-rate teleop;
  * arm stick axes as launch arguments. The defaults suit a generic Android
    gamepad: 0/1 left stick, 2/3 right stick, 4/5 triggers (rest at 1.0),
    6/7 d-pad.

ros2 launch vsv_teleop vsv_teleop.launch.py [max_speed:=1.5] [invert_steer:=false]
    [arm_lateral_axis:=2] [arm_vertical_axis:=3] [arm_forward_axis:=6]
    [deploy_button:=0] [park_button:=1] [arm_move_button:=2]

Start the simulation (Play) BEFORE this launch: vsv_driver and arm_ik give up
after waiting 60 s for the robot TF.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def arg(name, typ):
    return ParameterValue(LaunchConfiguration(name), value_type=typ)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_speed', default_value='1.5'),
        DeclareLaunchArgument('invert_steer', default_value='false'),
        DeclareLaunchArgument('arm_lateral_axis', default_value='2'),
        DeclareLaunchArgument('arm_vertical_axis', default_value='3'),
        DeclareLaunchArgument('arm_forward_axis', default_value='6'),
        # arm tip speed at full stick [m/s]; arm_ik clamps at arm_max_velocity
        DeclareLaunchArgument('arm_velocity', default_value='0.6'),
        DeclareLaunchArgument('arm_max_velocity', default_value='1.0'),
        # buttons: deploy tool (green), park tool (red), arm velocity mode (blue)
        DeclareLaunchArgument('deploy_button', default_value='0'),
        DeclareLaunchArgument('park_button', default_value='1'),
        DeclareLaunchArgument('arm_move_button', default_value='2'),
        # mux buttons: joystick / autonomous input for the truck and the arm.
        # 99 = disabled (both muxes stay on the joystick). Set real indices once
        # an autonomous controller publishes on /base_mux or /arm_mux/autoCommand.
        DeclareLaunchArgument('base_joy_button', default_value='99'),
        DeclareLaunchArgument('base_auto_button', default_value='99'),
        DeclareLaunchArgument('arm_joy_button', default_value='99'),
        DeclareLaunchArgument('arm_auto_button', default_value='99'),

        Node(package='joy', executable='joy_node', name='joy',
             parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05}],
             output='screen'),

        # Muxes: joystick input selected at startup. The *_joy_button and
        # *_auto_button arguments switch between joystick and /<x>_mux/autoCommand.
        Node(package='topic_tools', executable='mux', name='base_mux',
             arguments=['/vsv_driver/twistCommand', '/teleop_base/twistCommand',
                        '/base_mux/autoCommand'],
             parameters=[{'output_topic': '/vsv_driver/twistCommand',
                          'input_topics': ['/teleop_base/twistCommand', '/base_mux/autoCommand']}],
             remappings=[('/mux', '/base_mux')], output='screen'),
        Node(package='topic_tools', executable='mux', name='arm_mux',
             arguments=['/arm_ik/twist', '/teleop_arm/twistCommand', '/arm_mux/autoCommand'],
             parameters=[{'output_topic': '/arm_ik/twist',
                          'input_topics': ['/teleop_arm/twistCommand', '/arm_mux/autoCommand']}],
             remappings=[('/mux', '/arm_mux')], output='screen'),
        Node(package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_base_mux',
             parameters=[{'~/joystick_button': arg('base_joy_button', int),
                          '~/joystick_topic': '/teleop_base/twistCommand',
                          '~/auto_button': arg('base_auto_button', int),
                          '~/auto_topic': '/base_mux/autoCommand'}],
             remappings=[('select', '/base_mux/select')], output='screen'),
        Node(package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_arm_mux',
             parameters=[{'~/joystick_button': arg('arm_joy_button', int),
                          '~/joystick_topic': '/teleop_arm/twistCommand',
                          '~/auto_button': arg('arm_auto_button', int),
                          '~/auto_topic': '/arm_mux/autoCommand'}],
             remappings=[('select', '/arm_mux/select')], output='screen'),

        # Truck: left stick, car-like steering
        Node(package='vsv_teleop', executable='car_teleop', name='teleop_base',
             parameters=[{'axis_speed': 1, 'axis_steer': 0,
                          'max_speed': arg('max_speed', float),
                          'invert_steer': arg('invert_steer', bool),
                          'min_radius': 5.0}],
             output='screen'),

        # Arm tip: right stick; green = deploy, red = park, blue = velocity mode
        Node(package='vrep_vsv_driver', executable='teleop_geom', name='teleop_geom',
             parameters=[{'~/axis_arm_x': arg('arm_forward_axis', int),
                          '~/axis_arm_y': arg('arm_lateral_axis', int),
                          '~/axis_arm_z': arg('arm_vertical_axis', int),
                          '~/arm_velocity': arg('arm_velocity', float),
                          '~/home_button': arg('park_button', int),
                          '~/ready_button': arg('deploy_button', int),
                          '~/move_button': arg('arm_move_button', int)}],
             remappings=[('~/position_command', '/arm_ik/position'),
                         ('~/twist_command', '/teleop_arm/twistCommand'),
                         ('~/tool_command', '/arm_ik/tool_orientation')],
             output='screen'),

        Node(package='vrep_vsv_driver', executable='vsv_arm_ik', name='arm_ik',
             parameters=[{'~/max_velocity': arg('arm_max_velocity', float)}],
             remappings=[('~/joint_command', '/VSV/aggregated/command'),
                         ('~/joint_state', '/VSV/aggregated/state')],
             output='screen'),
        Node(package='vrep_vsv_driver', executable='vsv_arm', name='vsv_arm', output='screen'),
        Node(package='vrep_vsv_driver', executable='vsv_driver', name='vsv_driver',
             parameters=[{'~/min_radius': 5.0}], output='screen'),
    ])
