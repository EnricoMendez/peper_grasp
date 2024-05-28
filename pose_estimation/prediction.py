import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from ament_index_python import get_package_share_directory

class segmentator(Node):
    
    def __init__(self):
        super().__init__('segmentation')
        self.get_logger().info('segmentation initialized')

        # Create variables
        self.flag = False
        self.org_img = np.array((720, 1280, 3))
        self.pepper_detected = False
        
        # Define constants
        self.bridge = CvBridge()
        pkg_path = self.get_pkg_path(target='pose_estimation')
        print(pkg_path)
        model_path = pkg_path+'/best.pt'
        self.model = YOLO(model_path)  # load trained model
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)  
        
        # Create publishers
        self.pub_prediction = self.create_publisher(Image,'prediction',10)
        self.msg_prediction = Image()
        self.pub_mask = self.create_publisher(Image,'mask',10)
        self.msg_mask = Image()
        
        # Create subscribers
        self.camera_sub = self.create_subscription(Image,'/camera/color/image_raw', self.cam_callback, 10)
    
    def cam_callback(self,data):
        if not self.flag:
            self.get_logger().info("Image recieved", once=True)
            self.flag = True
        self.org_img = self.bridge.imgmsg_to_cv2(data)
        self.org_img = cv2.cvtColor(self.org_img, cv2.COLOR_BGR2RGB)

    def timer_callback(self):
        if not self.flag: return
        # Make prediction
        self.get_logger().info("Prediction ...", once=True)
        mask, prediction = self.get_pepper_mask(self.org_img)
        self.msg_prediction = self.bridge.cv2_to_imgmsg(prediction)
        self.msg_mask = self.bridge.cv2_to_imgmsg(mask)
        self.pub_prediction.publish(self.msg_prediction)
        if self.pepper_detected: self.pub_mask.publish(self.msg_mask)
    
    def get_pkg_path(self,target='size_estimation'):
        # Get exc path
        pkg_path = get_package_share_directory(target)

        # Converting to list
        parts = pkg_path.split('/')

        # Directing to the src folder
        replace = 'install'
        idx = parts.index(replace)
        parts[idx] = 'src'
        parts.remove('share')

        # Converting back to string
        path = '/'.join(parts)

        return path
    
    def get_pepper_mask(self,src):
        black_mask = np.zeros(src.shape[:2], dtype=np.uint8)
        results = self.model.predict(src, conf = 0.80, verbose = False)
        if results[0].masks is None: 
            self.pepper_detected = False
            return black_mask, src
        # iterate detection results 
        self.pepper_detected = True
        r = results[0]
        raw_prediction = r.plot()
        b_mask = np.zeros(src.shape[:2], np.uint8)
            # iterate each object contour 
        for ci,c in enumerate(r):
            # Create contour mask 
            contour = c.masks.xy.pop()
            contour = contour.astype(np.int32).reshape(-1, 1, 2)
            cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)


        # OPTION-1: Isolate object with black background
        mask3ch = cv2.cvtColor(b_mask, cv2.COLOR_GRAY2BGR)
        isolated = cv2.bitwise_and(mask3ch, src)
        return isolated, raw_prediction

def main(args=None):
    # Required lines for any node
    rclpy.init(args=args)
    node = segmentator()
    rclpy.spin(node)
    # Optional but good practices
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()