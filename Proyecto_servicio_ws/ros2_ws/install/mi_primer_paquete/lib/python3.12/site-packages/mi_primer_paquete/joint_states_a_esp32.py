#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3


class JointStatesAEsp32(Node):

    def __init__(self):
        super().__init__('joint_states_a_esp32')

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback_joint_states,
            10
        )

        self.pub = self.create_publisher(
            Vector3,
            '/servo_angles_rad',
            10
        )

    def callback_joint_states(self, msg):
        if len(msg.position) >= 3:
            out = Vector3()
            out.x = msg.position[0]
            out.y = msg.position[1]
            out.z = msg.position[2]

            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesAEsp32()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()