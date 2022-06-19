#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Script used to spawn a robot in a generic position."""

import argparse
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy

'''
python 利用 argparse 使程式執行時可帶參數
(https://www.wongwonggoods.com/python/python-argparse/)
'''

def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo with navigation2')
    parser.add_argument('-urdf', '--robot_urdf', type=str, default='neuronbot2.urdf',
                        help='Name of the robot to spawn')
    parser.add_argument('-n', '--robot_name', type=str, default='dummy_robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='dummy_robot_ns',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-namespace', '--namespace', type=bool, default=True,
                        help='Whether to enable namespacing')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    # sdf_file_path = os.path.join(
    #     get_package_share_directory('amazon_robot_gazebo'), 'models',
    #     'amazon_robot2', 'model.sdf')
    urdf_file_path = args.robot_urdf
    print('urdf_file_path:', str(urdf_file_path))
    
    # We need to remap the transform (/tf) topic so each robot has its own.
    # We do this by adding `ROS argument entries` to the urdf file for
    # each plugin broadcasting a transform. These argument entries provide the
    # remapping rule, i.e. /tf -> /<robot_id>/tf
    
    # 獲取 XML 文件物件 ElementTree
    tree = ET.parse(urdf_file_path)
    # 獲取 XML 文件物件的根結點 Element
    root = tree.getroot()
    imu_plugin = None
    diff_drive_plugin = None 
    #iter 可以在指定節點之下，以遞迴方式搜尋所有子節點：
    print("==============================")
    for plugin in root.iter('plugin'):
        #print(plugin.attrib)
        if 'differential_drive_controller' in plugin.attrib.values():
            diff_drive_plugin = plugin
            #print('************diff_drive_plugin:', diff_drive_plugin.attrib)
        elif 'box_bot_imu_plugin' in plugin.attrib.values():
            imu_plugin = plugin
            #print('************imu_plugin:', imu_plugin.attrib)
    
    # We change the namespace to the robots corresponding one
    tag_diff_drive_ros_params = diff_drive_plugin.find('ros')
    tag_diff_drive_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
    tag_diff_drive_ns.text = '/' + args.robot_namespace
    #print('************tag_diff_drive_ns:', tag_diff_drive_ns.text)
    ros_tf_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
    ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'
    #print('************ros_tf_remap:', ros_tf_remap.text)

    # if imu_plugin is not None:
    #     tag_imu_ros_params = imu_plugin.find('ros')
    #     tag_imu_ns = ET.SubElement(tag_imu_ros_params, 'namespace')
    #     tag_imu_ns.text = '/' + args.robot_namespace + '/imu'
    # else:
    #     print("ERROR>>>>>>>>>>>>>>>>>>>>> IMU NOT FOUND")


    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')
    #print("==============================")
    #print(ET.tostring(root, encoding='unicode'))
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)


    if args.namespace is True:
        node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.robot_name, args.robot_namespace, args.x, args.y, args.z))

        request.robot_namespace = args.robot_namespace
        print(args.robot_namespace)

    else:
        node.get_logger().info('spawning `{}` at {}, {}, {}'.format(
            args.robot_name, args.x, args.y, args.z))

    node.get_logger().info('Spawning Robot using service: `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
