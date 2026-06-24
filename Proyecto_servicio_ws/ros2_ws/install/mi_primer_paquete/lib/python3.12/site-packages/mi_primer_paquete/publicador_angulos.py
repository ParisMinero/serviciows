#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math


class PublicadorAngulos(Node):

    def __init__(self):
        super().__init__('publicador_angulos')

        self.publisher_ = self.create_publisher(
            Vector3,
            '/joint_angles_rad',
            10
        )

        self.timer = self.create_timer(1.0, self.publicar_angulos)

    def publicar_angulos(self):
        msg = Vector3()

        msg.x = math.radians(90)
        msg.y = math.radians(45)
        msg.z = math.radians(30)

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Publicado: q1={msg.x:.2f} rad, q2={msg.y:.2f} rad, q3={msg.z:.2f} rad'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PublicadorAngulos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()