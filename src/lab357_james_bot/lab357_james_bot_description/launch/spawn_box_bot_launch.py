from launch import LaunchDescription

#import launch.actions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    print('----------- into spawn_box_bot_launch.py -----------')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='lab357_james_bot_description',
            # executable='spawn_box_bot_v2.py',
            executable='nav2_gazebo_spawner.py',
            output='screen',
            arguments=[
                #'--robot_urdf', LaunchConfiguration('robot_urdf'),
                '--robot_name', LaunchConfiguration('robot_name'),
                '--robot_namespace', LaunchConfiguration('robot_namespace'),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '--turtlebot_type', LaunchConfiguration('turtlebot_type'),
            ]
        ),

    ])
