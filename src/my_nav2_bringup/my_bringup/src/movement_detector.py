#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64


class MovementDetectorNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("movement_detector_node")
        # _mved_distance is for stored distance moved
        # create and initialize it here. Initial value is 0.0
        self._mved_distance = Float64()
        self._mved_distance.data = 0.0
        # Get the inital position. This will be a reference point for calculating
        # the distance moved
        self._init_pose = False

        # Create a publisher for publishing the distance moved into the topic '/robot1/moved_distance'
        self.distance_moved_pub = self.create_publisher(
            Float64, "/robot1/moved_distance", 10)

        # create a subscriber for getting new Odometry messages
        self.create_subscription(
            Odometry, "/robot1/odom", self.odom_callback, 10)

    def get_init_position(self, msg):
        """Get the initial position of the robot."""
        data_odom = None
        # wait for a message from the odometry topic and store it in data_odom when available
        while data_odom is None:
            try:
                data_odom = msg
            except:
                self.get_logger().info("Current odom not ready yet, retrying for setting up init pose")

        # Store the received odometry "position" variable in a Point instance
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z
        self._init_pose = True
        self.get_logger().info("get init current_position!!!!!")

    def odom_callback(self, msg):
        if self._init_pose is False:
            self.get_init_position(msg)

        """Process odometry data sent by the subscriber."""
        # Get the position information from the odom message
        # See the structure of an /odom message in the `get_init_position` function
        NewPosition = msg.pose.pose.position

        # Calculate the new distance moved, and add it to _mved_distance and
        self._mved_distance.data += self.calculate_distance(
            NewPosition, self._current_position)

        # Update the current position of the robot so we have a new reference point
        # (The robot has moved and so we need a new reference for calculations)
        self.updatecurrent_positin(NewPosition)

        # If distance moved is big enough, publish it to the designated topic
        # Otherwise publish zero
        if self._mved_distance.data < 0.000001:
            aux = Float64()
            aux.data = 0.0
            self.distance_moved_pub.publish(aux)
        else:
            self.distance_moved_pub.publish(self._mved_distance)

    def updatecurrent_positin(self, new_position):
        """Update the current position of the robot."""
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def calculate_distance(self, new_position, old_position):
        """Calculate the distance between two Points (positions)."""
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist


def main(args=None):
    rclpy.init(args=args)
    node = MovementDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
