import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.transforms as transforms
import threading
import numpy as np
from midas.dpt_depth import DPTDepthModel
from midas.transforms import Resize, NormalizeImage, PrepareForNet


qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=5
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

        # Shared data and lock
        self.last_depth = None
        self.lock = threading.Lock()

        self.bridge = CvBridge()

        # Load MiDaS model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DPTDepthModel(
            "weights/dpt_hybrid_384.pt",
            backbone="vitb_rn50_384",
            non_negative=True
        )
        self.model.to(self.device)
        self.model.eval()

        # MiDaS transforms
        self.transform = transforms.Compose([
            Resize(384, 384),
            NormalizeImage(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            ),
            PrepareForNet()
        ])

        self.get_logger().info("MiDaS node initialized")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")
            return

        height, width = cv_image.shape[:2]
        dummy_mask = np.ones((height, width), dtype=np.uint8)

        input_batch = self.transform({"image": cv_image, "mask": dummy_mask})["image"]
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

        # Normalize depth
        depth_vis = (prediction - prediction.min()) / (prediction.max() - prediction.min())
        depth_vis = (depth_vis * 255).astype(np.uint8)

        # Publish as mono8 ROS image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_vis, encoding='mono8')
        self.depth_pub.publish(depth_msg)

        # Store last frame safely
        if len(depth_vis.shape) == 3:
    	    depth_vis = depth_vis.squeeze()

        with self.lock:
            self.last_depth = depth_vis


def main(args=None):
    rclpy.init(args=args)
    node = MiDaSNode()

    # Run ROS in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    print("Starting display loop... Press 'q' to quit.")
    try:
        while rclpy.ok():
            with node.lock:
                frame = node.last_depth.copy() if node.last_depth is not None else None

            if frame is not None:
                cv2.imshow("Depth Map", frame)
            else:
                # Wait a bit for the first frame
                cv2.waitKey(10)
                continue

            # Close on 'q'
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        ros_thread.join()


if __name__ == '__main__':
    main()

