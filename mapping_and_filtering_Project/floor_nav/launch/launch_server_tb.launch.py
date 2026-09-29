# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[
                {'autorepeat_rate': 10.},
                {'dev': "/dev/input/js0"},
                ],
            output='screen'),


        # launch_ros.actions.Node(
        #     package='collision_avoidance', executable='collision_avoidance', name='collision_avoidance',
        #     parameters=[
        #         {'~/safety_diameter': 0.5},
        #         {'~/ignore_diameter': 1.0},
        #         {'~/max_velocity': 0.3},
        #         {'~/only_forward': True},
        #         ],
        #     remappings=[
        #         # ('~/clouds', '/points'),
        #         ('~/scans', '/scan'),
        #         ('~/vel_input', '/mux/safeCommand'),
        #         ('~/vel_output', '/mux/autoCommand'),
        #         ],
        #     output='screen'),

        launch_ros.actions.Node(
            package='obstacle_avoidance', executable='obstacle_avoidance', name='obstacle_avoidance',
            parameters=[
                {'~/base_frame': 'base_link'},
                {'~/display': False},
                {"~/max_range": 3.0},
                {"~/max_linear_velocity": 0.1},
                {"~/max_angular_velocity": 0.3},
                {"~/max_linear_accel": 1.0},
                {"~/max_angular_accel": 5.0},
                {"~/map_resolution": 0.02},
                {"~/linear_velocity_resolution": 0.01},
                {"~/angular_velocity_resolution": 0.01},
                {"~/robot_radius": 0.3},
                {"~/time_horizon": 5.0},
                {"~/k_v": 1.0},
                {"~/k_w": 10.0},
                ],
            remappings=[
                ('~/scans', '/scan'),
                ('~/current_velocity', '/commands/velocity'),
                ('~/output_velocity', '/mux/autoCommand'),
                ('~/command_velocity', '/mux/safeCommand'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='topic_tools', executable='mux', name='cmd_mux',
            parameters=[
                {'output_topic': '/velocity_smoother/input'},
                {'input_topics': ['/teleop/twistCommand','/mux/autoCommand']},
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_node', name='teleop',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 0},
                {'~/scale_linear_x': 0.2},
                {'~/scale_angular': 1.},
                {'~/timeout': 1.0}
                ],
            remappings=[
                ('twistCommand', '/teleop/twistCommand'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_mux',
            parameters=[
                {'~/joystick_button': 0},
                {'~/joystick_topic': '/teleop/twistCommand'},
                {'~/auto_button': 1},
                {'~/auto_topic': '/mux/autoCommand'}
                ],
            remappings=[
                ('select', '/cmd_mux/select'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='floor_nav', executable='floornav_task_server', name='floor_tasks',
            parameters=[
                {'lib_path': os.path.join(os.getenv("HOME"),"ros2_ws/install/floor_nav/lib/floor_nav")},
                {'base_frame': 'base_link'},
                {'reference_frame': 'map'},
                ],
            remappings=[
                #('~/clouds3d', '/points'),
                ('~/scans', '/scan'),
                ('/mux/autoCommand', '/mux/safeCommand'),
                ],
            output='screen'),

    ])
