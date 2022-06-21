#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from osrf_pycommon.terminal_color import ansi

# ros2 launch box_bot_gazebo start_world_launch.py --show-args
# 可以顯示出DeclareLaunchArgument 中的定義


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_bot_gazebo = get_package_share_directory('lab357_james_bot_gazebo')
    pkg_bot_description = get_package_share_directory(
        'lab357_james_bot_description')
    gazebo_model_path = os.path.join(
        get_package_share_directory('lab357_james_bot_gazebo'), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_bot_gazebo, 'worlds', 'mememan_world.model'),
        description='Full path to world file to load')
    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    """ DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                pkg_bot_gazebo, 'worlds', 'world_only.model'), ''],
            description='SDF world file'), """

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    multi_bot_spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bot_description, 'launch',
                         'multi_spawn_robot_launch.py'),
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_simulator_cmd)
    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)
    ld.add_action(multi_bot_spawn_cmd)
    return ld

    """ return LaunchDescription([
        
        # Gazebo launch
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bot_description, 'launch',
                             'multi_spawn_robot_launch.py'),
            )
        )
    ]) """
