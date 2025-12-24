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
        
        # Create variable to store last image frames

        self.last_depth = None
        # create lock to prevent writing and reading happening at same time
        self.lock = threading.Lock()

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
        # # # self.get_logger().info("Published depth image")

	# lock so doesnt get read when writing to elf.last_depth
        with self.lock:
            self.last_depth = depth_vis

    def display_callback(self):
    	if self.last_depth is not None:
    	    cv2.imshow("Depth Map", depth_vis) # depth_msg is the message for topic, depth_vis is the actual image
    	    key = cv2.waitKey(1)
    	    if key == ord('q'):
    	        self.get_logger().info("Closing display windows")
    	        cv2.destroyAllWindows()
    	        rclpy.shutdown()
    	        
    	        
def display_loop(node: MiDaSNode):
    while rclpy.ok():
        with node.lock:
            #rgbframe = node.last_rgb.copy() if node.last_rgb is not None else None
            depthframe = node.last_depth.copy() if node.last_depth is not None else None
            
        # wait until first frame arrives with while loop
        while node.last_depth is None:
            print("Waiting for first depth frame...")
            rclpy.spin_once(node, timeout_sec=0.1)


        if depthframe is not None:
            print("Displaying frame...")
            try:
                print("Depth frame shape:", depthframe.shape, "dtype:", depthframe.dtype)
                cv2.imshow("Drone Camera Feed", depthframe)
            except cv2.error as e:
            	print(f"OpenCV error: {e}")
        else:
            print("No frame yet")

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = MiDaSNode()

    # Start display thread
    display_thread = threading.Thread( target=display_loop, args=(node,) )
    display_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        display_thread.join()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

