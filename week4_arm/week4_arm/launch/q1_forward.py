#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        self.L1 = 2.0
        self.L2 = 1.5

        self.publisher_ = self.create_publisher(Point, '/end_effector_position', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.get_logger().info('Forward Kinematics Node has been started.')

    def joint_states_callback(self, msg):
        if len(msg.position) < 2:
            self.get_logger().error('JointState message does not contain enough positions.')
            return

        theta1 = msg.position[0] + math.pi / 2.0
        theta2 = msg.position[1]

        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)

        end_effector_pos = Point()
        end_effector_pos.x = x
        end_effector_pos.y = y
        end_effector_pos.z = 0.0

        self.publisher_.publish(end_effector_pos)
        self.get_logger().info(f'Published End Effector Position: x={x:.3f}, y={y:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
