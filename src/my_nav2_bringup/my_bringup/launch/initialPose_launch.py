from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    print('------------------initialPose_launch---------------------------')
    bringup_dir = get_package_share_directory('my_bringup')
    initial_pose_1_cmd = Node(
        package='my_bringup',
        executable='initialPose.py',
        name='initialPose_1',
        output='screen',
        parameters=[{'robot_namespace': 'robot1'},
                    {'init_x_pose': 0.05},
                    {'init_y_pose': 0.5}]
    )

    initial_pose_2_cmd = Node(
        package='my_bringup',
        executable='initialPose.py',
        name='initialPose_2',
        output='screen',
        parameters=[{'robot_namespace': 'robot2'},
                    {'init_x_pose': 0.05},
                    {'init_y_pose': -0.5}]
    )

    ld = LaunchDescription()
    ld.add_action(initial_pose_1_cmd)
    ld.add_action(initial_pose_2_cmd)
    return ld
