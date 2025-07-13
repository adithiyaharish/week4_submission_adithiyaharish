#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        self.L1 = 2.0
        self.L2 = 1.5
        self.current_x = None
        self.current_y = None

        self.create_subscription(Point, '/end_effector_position', self.position_callback, 10)
        self.joint_angles_publisher = self.create_publisher(Float64MultiArray, '/joint_angles_goal', 10)

        self.get_logger().info('Inverse Kinematics Node Started. Waiting for end-effector position...')
    
    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.get_logger().info(f'Current end-effector position received: x={msg.x:.3f}, y={msg.y:.3f}')
        self.get_user_input_and_compute()

    def get_user_input_and_compute(self):
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn('No current position available yet.')
            return

        direction = input("Enter direction to move ('x' or 'y'): ").strip().lower()
        if direction not in ['x', 'y']:
            self.get_logger().error("Invalid direction. Enter 'x' or 'y'.")
            return

        try:
            distance = float(input("Enter distance to move (max 0.5m): "))
            if abs(distance) > 0.5:
                self.get_logger().warn("Distance exceeds 0.5m. Limiting to 0.5m.")
                distance = 0.5 if distance > 0 else -0.5
        except ValueError:
            self.get_logger().error("Invalid distance input.")
            return

        target_x = self.current_x
        target_y = self.current_y

        if direction == 'x':
            target_x += distance
        elif direction == 'y':
            target_y += distance

        self.get_logger().info(f'Target position: x={target_x:.3f}, y={target_y:.3f}')

        if not self.is_reachable(target_x, target_y):
            self.get_logger().warn('Target position is unreachable. Skipping computation.')
            return

        theta1, theta2 = self.compute_inverse_kinematics(target_x, target_y)

        joint_angles_msg = Float64MultiArray()
        joint_angles_msg.data = [theta1, theta2]
        self.joint_angles_publisher.publish(joint_angles_msg)

        self.get_logger().info(f'Published joint angles: theta1={theta1:.3f} rad, theta2={theta2:.3f} rad')

    def is_reachable(self, x, y):
        distance = math.hypot(x, y)
        return distance <= (self.L1 + self.L2)

    def compute_inverse_kinematics(self, x, y):
        # Compute using 2-link planar IK formula
        distance_squared = x**2 + y**2
        cos_theta2 = (distance_squared - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)

        # Clamp due to numerical errors
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
        theta2 = math.acos(cos_theta2)

        k1 = self.L1 + self.L2 * math.cos(theta2)
        k2 = self.L2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        return theta1, theta2


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
