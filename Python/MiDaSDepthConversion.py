import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
import numpy as np
from midas.dpt_depth import DPTDepthModel
from midas.transforms import Resize, NormalizeImage, PrepareForNet

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=5
)

class MiDaSDepthPublisher(Node):
    def __init__(self):
        super().__init__('midas_depth_publisher')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth_image',
            10
        )

        self.bridge = CvBridge()

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DPTDepthModel(
            "weights/dpt_hybrid_384.pt",
            backbone="vitb_rn50_384",
            non_negative=True
        )
        self.model.to(self.device)
        self.model.eval()

        self.transform = transforms.Compose([
            Resize(384, 384),
            NormalizeImage(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            ),
            PrepareForNet()
        ])

        self.get_logger().info("MiDaS Depth Publisher initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"CV bridge failure: {e}")
            return

        height, width = cv_image.shape[:2]
        dummy_mask = np.ones((height, width), dtype=np.uint8)
        input_batch = self.transform({"image": cv_image, "mask": dummy_mask})["image"]
        input_batch = torch.from_numpy(input_batch).unsqueeze(0).to(self.device)

        with torch.no_grad():
            prediction = self.model.forward(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=cv_image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze().cpu().numpy()

        delta = prediction.max() - prediction.min()
        if delta > 0:
            depth_vis = (prediction - prediction.min()) / delta
        else:
            depth_vis = np.zeros_like(prediction)
        depth_vis = (depth_vis * 255).astype(np.uint8)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_vis, encoding='mono8')
        self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MiDaSDepthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

