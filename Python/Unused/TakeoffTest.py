#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class TelloTakeoffLandTest(Node):
    def __init__(self):
        super().__init__('tello_takeoff_land_test')
        self.takeoff_pub = self.create_publisher(Empty, 'takeoff', 10)
        self.land_pub = self.create_publisher(Empty, 'land', 10)

    def send_takeoff(self):
        self.get_logger().info('Sending TAKEOFF')
        self.takeoff_pub.publish(Empty())

    def send_land(self):
        self.get_logger().info('Sending LAND')
        self.land_pub.publish(Empty())

def main():
    rclpy.init()
    node = TelloTakeoffLandTest()

    # Give time for connections
    time.sleep(1.0)

    # Takeoff
    node.send_takeoff()

    # Wait a few seconds in the air
    time.sleep(5.0)

    # Land
    node.send_land()

    # Wait a bit so messages are sent before shutdown
    time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

