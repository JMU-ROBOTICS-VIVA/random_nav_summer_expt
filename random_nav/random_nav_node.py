#!/usr/bin/env python

"""This node repeatedly selects random map positions until it finds
  one that can be navigated to.

  It then navigates to the random goals using the ROS navigation stack.

"""
import numpy as np
import rclpy
from rclpy.node import Node
import map_utils

from nav_msgs.msg import OccupancyGrid


class RandomNavNode(object):
    def __init__(self):
        super().__init__('random_nav')

        self.subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        self.map_msg = None

        # Need help
        while self.map_msg is None:
            rospy.loginfo("Waiting for map...")
            rospy.sleep(.1)

        self.map = map_utils.Map(self.map_msg)

        self.demo_map()

        # REPEAT THE FOLLOWING UNTIL ROSPY IS SHUT DOWN:
        #
        #    GENERATE A RANDOM GOAL LOCATION:
        #       *GENERATE RANDOM REAL-VALUED LOCATIONS WHERE X AND Y ARE BOTH
        #        IN THE RANGE [-10, 10].
        #       *CONTINUE GENERATING RANDOM GOAL LOCATIONS UNTIL ONE IS
        #        AT A FREE LOCATION IN THE MAP (see demo_map below)
        #
        #    ATTEMPT TO NAVIGATE TO THE GOAL:
        #       * SEND THE DESIRED GOAL IN THE MAP COORDINATE FRAME
        #         (see provided action_nav.py file)

    def demo_map(self):
        """ Illustrate how to interact with a loaded map object. """
        x_pos = 0.0
        y_pos = -2.0

        if self.map.get_cell(x_pos, y_pos) == 0:
            message = "clear"
        elif self.map.get_cell(x_pos, y_pos) == -1:
            message = "unknown"
        elif np.isnan(self.map.get_cell(x_pos, y_pos)):
            message = "unknown (out of bounds)"
        else:
            message = "occupied"

        message = "Position ({}, {}) is ".format(x_pos, y_pos) + message
        rospy.loginfo(message)

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg


def main(args=None):
    rclpy.init(args=args)
    node = RandomNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.is_shutdown()


if __name__ == "__main__":
    main()
