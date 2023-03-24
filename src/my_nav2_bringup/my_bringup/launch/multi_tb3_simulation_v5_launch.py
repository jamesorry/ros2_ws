# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example for spawing multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node


def gen_robot_list(number_of_robots):
    robots = []
    # 這裡修改座標，initialPose.py也要改
    # if number_of_robots is 1:
    #     robots.append({'name': "robot1", 'x_pose': 0.0,
    #                   'y_pose': 0.5, 'z_pose': 0.01})
    # ===========================================
    # elif number_of_robots is 2: # this  is for position world_house.model
    #     robots.append({'name': "robot1", 'x_pose': 1.0,
    #                   'y_pose': 1.5, 'z_pose': 0.01})
    #     robots.append({'name': "robot2", 'x_pose': -1.0,
    #                   'y_pose': 1.5, 'z_pose': 0.01})

    # 2022/09/30 modify===========================================
    # elif number_of_robots is 2:  # this  is for position world_only.model
    #     robots.append({'name': "robot1", 'x_pose': 0.0,
    #                   'y_pose': 0.5, 'z_pose': 0.01})
    #     robots.append({'name': "robot2", 'x_pose': 0.0,
    #                   'y_pose': -0.5, 'z_pose': 0.01})
    # 修改啟動時座標的設定方式
    if number_of_robots is 1:
        robots.append({'name': "robot1", 'x_pose': 1.5,
                      'y_pose': -3.0, 'z_pose': 0.01})
    elif number_of_robots is 2:  # this  is for position my_own_world_only.model
        # robots.append({'name': "robot1", 'x_pose': -2.0,
        #               'y_pose': 1.0, 'z_pose': 0.01})
        robots.append({'name': "robot1", 'x_pose': -4.0,
                      'y_pose': -5.0, 'z_pose': 0.01})
        robots.append({'name': "robot2", 'x_pose': 0.0,
                      'y_pose': -1.5, 'z_pose': 0.01})
    # 2022/09/30 modify===========================================
    return robots


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('my_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    robots = gen_robot_list(1)

    # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')
    # slam = LaunchConfiguration('slam')
    # slam_toolbox = LaunchConfiguration("slam_toolbox")
    # slam_gmapping = LaunchConfiguration("slam_gmapping")
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='True')

    # Declare the launch arguments
    # declare_world_cmd = DeclareLaunchArgument(
    #     'world',
    #     default_value=os.path.join(
    #         bringup_dir, 'worlds', 'world_house.model'),
    #     description='Full path to world file to load')

    # 2022/09/30 modify===========================================
    # declare_world_cmd = DeclareLaunchArgument(
    #     'world',
    #     default_value=os.path.join(
    #         bringup_dir, 'worlds', 'world_only.model'),
    #     description='Full path to world file to load')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            bringup_dir, 'worlds', 'my_own_world_v2_only.model'),
        description='Full path to world file to load')
    # 2022/09/30 modify===========================================

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(
    #         bringup_dir, 'maps', 'waffle_house.yaml'),
    #     description='Full path to map file to load')

    # 2022/09/30 modify===========================================
    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(
    #         bringup_dir, 'maps', 'turtlebot3_world.yaml'),
    #     description='Full path to map file to load')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'my_own_map_v3.yaml'),
        description='Full path to map file to load')
    # 2022/09/30 modify===========================================

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        'robot1_params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'nav2_fix_multirobot_params_1_v2.yaml'), # 2022/09/30 modify
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        'robot2_params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'nav2_fix_multirobot_params_2_v2.yaml'), # 2022/09/30 modify
        description='Full path to the ROS2 parameters file to use for robot2 launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the stacks')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_namespaced_view_fix.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # declare_slam_cmd = DeclareLaunchArgument(
    #     'slam',
    #     default_value='False',
    #     description='Whether run a SLAM')
    # declare_slam_toolbox_cmd = DeclareLaunchArgument(
    #     "slam_toolbox", default_value="False", description="Whether run a SLAM toolbox"
    # )
    # declare_slam_gmapping_cmd = DeclareLaunchArgument(
    #     "slam_gmapping",
    #     default_value="False",
    #     description="Whether run a SLAM gmapping",
    # )

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator,
             '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world],
        output='screen')

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch',
                                                           'spawn_tb3_launch.py')),
                launch_arguments={
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                    'robot_name': robot['name'],
                    'turtlebot_type': TextSubstitution(text='waffle')
                }.items()))

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(robot['name'] + '_params_file')

        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        'namespace': TextSubstitution(text=robot['name']),
                        'use_namespace': 'True',
                        'rviz_config': rviz_config_file}.items()),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                               'launch',
                                                               'tb3_simulation_launch.py')),
                    launch_arguments={'namespace': robot['name'],
                                      'use_namespace': 'True',
                                      'map': map_yaml_file,
                                      'use_sim_time': 'True',
                                      'params_file': params_file,
                                      'default_bt_xml_filename': default_bt_xml_filename,
                                      'autostart': autostart,
                                      'use_rviz': 'False',
                                      'use_simulator': 'False',
                                      'headless': 'False',
                                      #   "slam": slam,
                                      #   "slam_toolbox": slam_toolbox,
                                      #   "slam_gmapping": slam_gmapping,
                                      'use_robot_state_pub': use_robot_state_pub}.items()),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=['Start of loginfo------multi_tb3_simulation_launch.py']),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=['Launching ', robot['name']]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' map yaml: ', map_yaml_file]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' params yaml: ', params_file]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' behavior tree xml: ', default_bt_xml_filename]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' autostart: ', autostart]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=['End of loginfo------multi_tb3_simulation_launch.py']),
            ]
        )
        nav_instances_cmds.append(group)
    
    init_pose_array = []
    robot_num = 0
    for robot in robots:
        robot_num = robot_num + 1
        init_pose_array.append(robot['x_pose'])
        init_pose_array.append(robot['y_pose'])
    
    initialPose_cmd = Node(
        # condition=IfCondition(PythonExpression(["not ", slam, " and not ", slam_gmapping])),
        package='my_bringup',
        executable='initialPose_v3.py',
        parameters=[
            {'robot_num': robot_num},
            {'init_pose_array': init_pose_array},
        ],
        name='creat_initialPose',
        output='screen'
    )
    bt_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bt_ros2'), 'launch', 'bt_ros2.launch.py'))
    )

    declare_map_pgm_cmd = DeclareLaunchArgument(
        'map_pgm',
        # default_value=os.path.join(
        #     bringup_dir, 'maps', 'my_own_map_v3.pgm'), # 2023/03/17 modify
        #     # bringup_dir, 'maps', 'my_own_map_v2.pgm'), # 2022/09/30 modify
        default_value = "/robot1", # 2023/03/25 modify
        description='Full path to pgm map to load')

    # run SendMapImage node
    send_map_image_cmd = Node(
        package='qt_publish',
        executable='SendMapImage_v2.py',
        output='screen',
        arguments=['--map', LaunchConfiguration('map_pgm')]
    )
    """ 
    TimerAction() 函数可以在指定的时间后执行一个action, 需要接受参数有
        period:接受一个float, 延迟的时间
        actions:接受一个list, [action_1, action_2,…], 列表中装要执行的action 
    """
    timer_action_1_cmd = TimerAction(period=10.0, actions=[initialPose_cmd])
    timer_action_2_cmd = TimerAction(period=12.0, actions=[send_map_image_cmd])
    timer_action_3_cmd = TimerAction(period=15.0, actions=[bt_nav_cmd])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    # ld.add_action(declare_slam_cmd)
    # ld.add_action(declare_slam_toolbox_cmd)
    # ld.add_action(declare_slam_gmapping_cmd)
    ld.add_action(declare_map_pgm_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    ld.add_action(timer_action_1_cmd)
    ld.add_action(timer_action_2_cmd)
    return ld
