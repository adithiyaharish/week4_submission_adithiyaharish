#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class RoverControlTester(Node):
    def __init__(self):
        super().__init__('rover_control_tester')
        
        # Increase timer period to 8 seconds
        self.timer = self.create_timer(4.0, self.send_test_command)
        self.command_index = 0
        
        self.control_pub = self.create_publisher(
            Float64MultiArray,
            '/rover/control_input',
            10
        )
        
        self.get_logger().info('Rover Control Tester Started')
        self.get_logger().info('Publishing 8-element test commands to /rover/control_input')
    
    def send_test_command(self):
        msg = Float64MultiArray()
        
        if self.command_index == 0:
            msg.data = [0.0] * 8
            self.get_logger().info('Test 0: All zeros (reset/stop)')
        elif self.command_index == 1:
            msg.data = [0.2, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info('Test 1: All wheels steering 0.2 rad, no drive')
        elif self.command_index == 2:
            msg.data = [0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 2.0]
            self.get_logger().info('Test 2: No steering, all wheels driving at 2.0 rad/s')
        elif self.command_index == 3:
            msg.data = [0.3, 0.3, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0]
            self.get_logger().info('Test 3: Front steering 0.3 rad, left wheels forward, right wheels backward')
        elif self.command_index == 4:
            msg.data = [-0.2, -0.2, -0.2, -0.2, -1.5, -1.5, -1.5, -1.5]
            self.get_logger().info('Test 4: All wheels steering -0.2 rad, driving at -1.5 rad/s')
        else:
            msg.data = [0.0] * 8
            self.get_logger().info('Test 5: Reset to all zeros')
            self.command_index = -1
        
        self.control_pub.publish(msg)
        self.command_index += 1

def main(args=None):
    rclpy.init(args=args)
    tester = RoverControlTester()
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
