import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np

class ObstacleAveragesNode(Node):
    def __init__(self):
        super().__init__('obstacle_averages_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            10
        )
        
        # data[0] = front_avg, data[1] = left_avg, data[2] = right_avg
        self.avg_pub = self.create_publisher(
            Float32MultiArray,
            '/haptic/obstacle_averages',
            10
        )
        
        self.bridge = CvBridge()
        
        self.CENTER_W = 85
        self.CENTER_H = 48
        
        self.get_logger().info("ObstacleAveragesNode ready (publishing avg intensities)")

    def depth_callback(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            h, w = depth_img.shape

            # --- Front region (center) ---
            cy1 = h // 2 - self.CENTER_H // 2
            cy2 = h // 2 + self.CENTER_H // 2
            cx1 = w // 2 - self.CENTER_W // 2
            cx2 = w // 2 + self.CENTER_W // 2
            center_region = depth_img[cy1:cy2, cx1:cx2]
            center_avg = float(np.mean(center_region))

            # --- Left / right regions (leftmost/rightmost 1/8 of width) ---
            wall_w = w // 8
            left_region = depth_img[:, :wall_w]
            right_region = depth_img[:, -wall_w:]

            left_avg = float(np.mean(left_region))
            right_avg = float(np.mean(right_region))

            # Publish raw averages (0–255)
            out = Float32MultiArray()
            out.data = [center_avg, left_avg, right_avg]
            self.avg_pub.publish(out)

            print(
                f"front_avg={center_avg:.1f}, left_avg={left_avg:.1f}, right_avg={right_avg:.1f}"
            )

        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAveragesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

