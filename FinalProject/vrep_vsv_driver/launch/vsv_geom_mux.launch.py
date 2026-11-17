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

        launch_ros.actions.Node(
            package='topic_tools', executable='mux', name='base_mux',
            arguments=['/vsv_driver/twistCommand','/teleop_base/twistCommand','/base_mux/autoCommand'],
            parameters=[
                {'output_topic': '/vsv_driver/twistCommand'},
                {'input_topics': ['/teleop_base/twistCommand','/base_mux/autoCommand']},
                ],
            remappings=[
                ('/mux', '/base_mux'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='topic_tools', executable='mux', name='arm_mux',
            arguments=['/arm_ik/twist','/teleop_arm/twistCommand','/arm_mux/autoCommand'],
            parameters=[
                {'output_topic': '/arm_ik/twist'},
                {'input_topics': ['/teleop_arm/twistCommand','/arm_mux/autoCommand']},
                ],
            remappings=[
                ('/mux', '/arm_mux'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_base_mux',
            parameters=[
                {'~/joystick_button': 4},
                {'~/joystick_topic': '/teleop_base/twistCommand'},
                {'~/auto_button': 5},
                {'~/auto_topic': '/base_mux/autoCommand'},
                ],
            remappings=[
                ('select', '/base_mux/select'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_arm_mux',
            parameters=[
                {'~/joystick_button': 6},
                {'~/joystick_topic': '/teleop_arm/twistCommand'},
                {'~/auto_button': 7},
                {'~/auto_topic': '/arm_mux/autoCommand'},
                ],
            remappings=[
                ('select', '/arm_mux/select'),
                ],
            output='screen'),


        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_node', name='teleop_base',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 0},
                {'~/scale_linear_x': 1.0},
                {'~/scale_angular': 1.},
                {'~/timeout': 1.0}
                ],
            remappings=[
                #('twistCommand', '/teleop/twistCommand'),
                ],
            output='screen'),


        launch_ros.actions.Node(
            package='vrep_vsv_driver', executable='vsv_arm_ik', name='arm_ik',
            parameters=[
                {'~/max_velocity': 0.5},
                ],
            remappings=[
                ('~/joint_command', '/VSV/aggregated/command'),
                ('~/joint_state', '/VSV/aggregated/state'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_vsv_driver', executable='teleop_geom', name='teleop_geom',
            parameters=[
                {'~/axis_arm_x': 2},
                {'~/axis_arm_y': 4},
                {'~/axis_arm_z': 3},
                {'~/arm_velocity': 0.25},
                {'~/home_button': 1},
                {'~/ready_button': 0},
                {'~/move_button': 2},
                ],
            remappings=[
                ('~/position_command', '/arm_ik/position'),
                ('~/twist_command', '/teleop_arm/twistCommand'),
                ('~/tool_command', '/arm_ik/tool_orientation'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_vsv_driver', executable='vsv_arm', name='vsv_arm',
            parameters=[
                ],
            remappings=[
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_vsv_driver', executable='vsv_driver', name='vsv_driver',
            parameters=[
                {'~/min_radius': 5.0},
                ],
            remappings=[
                ],
            output='screen'),


    ])
