#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math


class RoverControlNode(Node):
    def __init__(self):
        super().__init__('rover_control_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/rover/control_input', 10)
        self.get_logger().info('Rover control node started.')

        self.execute_sequence()

    def publish_control(self, steer_angles, drive_speeds):
        msg = Float64MultiArray()
        msg.data = steer_angles + drive_speeds
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published control: {msg.data}")

    def execute_sequence(self):
        # ---- STEP 1: Move forward 2 meters ----
        steer_angles = [0.0, 0.0, 0.0, 0.0]  # no steering
        drive_speeds = [1.0, 1.0, 1.0, 1.0]  # 1 m/s
        self.get_logger().info("Moving forward 2 meters")
        self.publish_control(steer_angles, drive_speeds)
        time.sleep(2)  # Assuming 1m/s -> 2 meters in 2 seconds

        # ---- STOP ----
        self.publish_control(steer_angles, [0.0]*4)
        time.sleep(1)

        # ---- STEP 2: Rotate wheels -90 deg (left turn) ----
        steer_angles = [-math.pi/2]*4  # -90 degrees
        self.publish_control(steer_angles, [0.0]*4)
        self.get_logger().info("Rotated wheels to -90 degrees")
        time.sleep(1)

        # ---- STEP 3: Move forward 2 meters (leftward due to steering) ----
        drive_speeds = [1.0]*4
        self.get_logger().info("Moving leftward 2 meters")
        self.publish_control(steer_angles, drive_speeds)
        time.sleep(2)

        # ---- STOP ----
        self.publish_control(steer_angles, [0.0]*4)
        time.sleep(1)

        # ---- STEP 4a: Rotate in-place using Differential Drive ----
        steer_angles = [0.0]*4
        drive_speeds = [1.0, 1.0, -1.0, -1.0]  # left wheels fwd, right wheels back
        self.get_logger().info("Rotating in-place using differential drive")
        self.publish_control(steer_angles, drive_speeds)
        time.sleep(2)

        # ---- STOP ----
        self.publish_control(steer_angles, [0.0]*4)
        time.sleep(1)

        # ---- STEP 4b: Rotate in-place using Independent Steering ----
        # Wheels steered to form a point turn (~45 degrees inward)
        steer_angles = [math.pi/4, -math.pi/4, -math.pi/4, math.pi/4]
        drive_speeds = [1.0]*4
        self.get_logger().info("Rotating in-place using independent steering")
        self.publish_control(steer_angles, drive_speeds)
        time.sleep(2)

        # ---- STOP ----
        self.publish_control(steer_angles, [0.0]*4)
        time.sleep(1)

        # ---- STEP 5: All wheels 45 degrees and move diagonally ----
        steer_angles = [math.pi/4]*4
        drive_speeds = [1.0]*4
        self.get_logger().info("Moving diagonally with all wheels at 45 degrees")
        self.publish_control(steer_angles, drive_speeds)
        time.sleep(2)

        # ---- Final STOP ----
        self.publish_control(steer_angles, [0.0]*4)
        self.get_logger().info("Completed all maneuvers.")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

if __name__ == '__main__':
    rclpy.init()
    node = RoverControlNode()
    node.execute_sequence()  # assuming you have a function that runs steps
    node.destroy_node()
    rclpy.shutdown()
