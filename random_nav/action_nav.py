#!/usr/bin/env python
"""Demo of sending navigation goals using actionlib.

Moves the robot to a designated point relative to the its current
position.

Usage:

action_nav.py TARGET_X TARGET_Y [THETA]

Author: Nathan Sprague
Version: 2/14/2019

"""
import sys

import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
import transform_util

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus


def create_goal_message(x_target, y_target, theta_target, frame='map'):
    """Create a goal message in the indicated frame"""

    quat = transform_util.transformations.quaternion_from_euler(
        0, 0, theta_target)
    # Create a goal message ...
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rclpy.timer.Clock.now()
    goal.target_pose.pose.position.x = x_target
    goal.target_pose.pose.position.y = y_target
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    return goal


class NavNode(Node):

    def __init__(self):
        """ Set up the node. """
        super().__init__('nav_node')
        self.ac = ActionClient(self, MoveBaseAction, "move_base")

    def goto_point(self, x_target, y_target, theta_target=0):
        """ Move to a location relative to the robot's current position """

        self.get_logger().info("navigating to: ({},{})".format(x_target, y_target))

        goal = create_goal_message(x_target, y_target, theta_target,
                                   'base_link')

        self.get_logger().info("Waiting for server.")
        self.ac.wait_for_server()

        self.get_logger().info("Sending goal.")
        self.ac.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.get_logger().info("Goal Sent.")

        # Wait until the server reports a result.
        self.ac.wait_for_result()
        self.get_logger().info("Status Text: {}".format(self.ac.get_goal_status_text()))

        # Should be either "SUCCEEDED" or "ABORTED"
        state_name = actionlib.get_name_of_constant(GoalStatus,
                                                    self.ac.get_state())
        self.get_logger().info("State      : {}".format(state_name))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Received feedback: {0}".format(feedback.partial_sequence)

def main(args=None):
    rclpy.init(args=args)
    node=NavNode(args)
    rclpy.spin(node)
    if len(sys.argv) == 3:
        node.goto_point(float(sys.argv[1]), float(sys.argv[2]))
    else:
        node.goto_point(float(sys.argv[1]), float(sys.argv[2]),
                            float(sys.argv[3]))
    node.destroy_node()
    rclpy.is_shutdown()


if __name__ == "__main__":
    main()
