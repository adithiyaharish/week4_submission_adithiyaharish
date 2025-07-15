#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class RoverControlNode(Node):
    def __init__(self):
        super().__init__('rover_control_node')

        # Publishers for steering (position) and drive (velocity)
        self.steering_pubs = [
            self.create_publisher(Float64MultiArray, '/front_left_steering_controller/commands', 10),
            self.create_publisher(Float64MultiArray, '/front_right_steering_controller/commands', 10),
            self.create_publisher(Float64MultiArray, '/rear_left_steering_controller/commands', 10),
            self.create_publisher(Float64MultiArray, '/rear_right_steering_controller/commands', 10)
        ]
        self.drive_pubs = [
            self.create_publisher(Float64MultiArray, '/front_left_drive_controller/commands', 10),
            self.create_publisher(Float64MultiArray, '/front_right_drive_controller/commands', 10),
            self.create_publisher(Float64MultiArray, '/rear_left_drive_controller/commands', 10),
            self.create_publisher(Float64MultiArray, '/rear_right_drive_controller/commands', 10)
        ]

        # Subscribe to control input
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/rover/control_input',
            self.control_callback,
            10
        )

        self.get_logger().info('Rover Control Node Started')
        self.get_logger().info('Listening to /rover/control_input topic')
        self.get_logger().info(
            'Expected input: [fl_steer, fr_steer, rl_steer, rr_steer, fl_drive, fr_drive, rl_drive, rr_drive]'
        )

        # Send zero commands initially
        self.send_zero_commands()

    def send_zero_commands(self):
        zero_steering = [0.0] * 4
        zero_drive = [0.0] * 4
        self.publish_commands(self.steering_pubs, zero_steering, 'steering (initial)')
        self.publish_commands(self.drive_pubs, zero_drive, 'drive (initial)')
        self.get_logger().info('Sent initial zero commands to all joints.')

    def control_callback(self, msg):
        data = msg.data
        self.direct_joint_control(data)

    def direct_joint_control(self, joint_commands):
        # [fl_steer, fr_steer, rl_steer, rr_steer, fl_drive, fr_drive, rl_drive, rr_drive]
        steering = joint_commands[:4]
        drives = joint_commands[4:]

        self.publish_commands(self.steering_pubs, steering, 'steering')
        self.publish_commands(self.drive_pubs, drives, 'drive')

        self.get_logger().info(
            f'Direct control - Steering: {["%.2f" % s for s in steering]}'
        )
        self.get_logger().info(
            f'Direct control - Drive: {["%.2f" % d for d in drives]}'
        )

    def publish_commands(self, publishers, values, label):
        for i, (pub, value) in enumerate(zip(publishers, values)):
            msg = Float64MultiArray()
            msg.data = [value]
            pub.publish(msg)
            self.get_logger().debug(f'Published {label} command {i}: {value:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
