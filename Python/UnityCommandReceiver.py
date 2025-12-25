#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty
from geometry_msgs.msg import Twist

class UnityCommandBridge(Node):
    def __init__(self):
        super().__init__('unity_command_bridge')

        self.sub = self.create_subscription(
            Float32MultiArray,
            'unity_cmd',   # matches Unity unifiedTopic
            self.callback,
            10
        )

        self.pub_control = self.create_publisher(Twist, 'control', 10)
        self.pub_takeoff = self.create_publisher(Empty, 'takeoff', 10)
        self.pub_land    = self.create_publisher(Empty, 'land', 10)

        self.get_logger().info("UnityCommandBridge ready")

    def callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 4:
            self.get_logger().warn("unity_cmd msg too short")
            return

        forward = float(data[0])   # -100..100 RC pitch [web:71]
        yaw     = float(data[1])   # -100..100 RC yaw   [web:71]
        takeoff_flag = data[2]
        land_flag    = data[3]

        # Clamp to Tello RC range
        forward = max(-100.0, min(100.0, forward))
        yaw     = max(-100.0, min(100.0, yaw))

        twist = Twist()
        twist.linear.x  = 0.0
        twist.linear.y  = forward
        twist.linear.z  = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = yaw
        self.pub_control.publish(twist)

        if takeoff_flag > 0.5:
            self.get_logger().info("Received TAKEOFF from Unity")
            self.pub_takeoff.publish(Empty())

        if land_flag > 0.5:
            self.get_logger().info("Received LAND from Unity")
            self.pub_land.publish(Empty())

def main(args=None):
    rclpy.init(args=args)
    node = UnityCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
