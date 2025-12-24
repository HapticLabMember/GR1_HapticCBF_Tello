#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
import cv2
import numpy as np
from midas.dpt_depth import DPTDepthModel  # adjust import if using MiDaS v3
from midas.transforms import Resize, NormalizeImage, PrepareForNet

qos_profile = QoSProfile(
	reliability = ReliabilityPolicy.BEST_EFFORT,
	depth = 10
)

class MiDaSNode(Node):
    def __init__(self):
        super().__init__('midas_depth_node')
        
        # ROS 2 subscriptions and publishers
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # topic from drone
            self.image_callback,
            qos_profile
        )
        
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth_image',
            10
        )

        self.bridge = CvBridge()
        
        # Load MiDaS model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DPTDepthModel("weights/dpt_hybrid_384.pt", backbone="vitb_rn50_384", non_negative=True)
        self.model.to(self.device)
        self.model.eval()
        
        # MiDaS transforms
        self.transform = transforms.Compose([
            Resize(
                384, 384
            ),
            NormalizeImage(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            ),
            PrepareForNet()
        ])
        
        self.get_logger().info("MiDaS node initialized")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        
        height, width = cv_image.shape[:2]
        dummy_mask = np.ones((height,width), dtype=np.uint8)
        
        input_batch = self.transform({
        	"image": cv_image,
        	"mask": dummy_mask
        })["image"]
        input_batch = torch.from_numpy(input_batch).unsqueeze(0).to(self.device)

        # Run inference
        with torch.no_grad():
            prediction = self.model.forward(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=cv_image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze().cpu().numpy()
        
        # Normalize depth for visualization
        depth_vis = (prediction - prediction.min()) / (prediction.max() - prediction.min())
        depth_vis = (depth_vis * 255).astype(np.uint8)
        
        # Publish depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_vis, encoding='mono8')
        self.depth_pub.publish(depth_msg)
        self.get_logger().info("Published depth image")

def main(args=None):
    rclpy.init(args=args)
    node = MiDaSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
