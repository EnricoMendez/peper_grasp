import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import std_msgs.msg
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

class RGBDPointCloudGenerator(Node):
    def __init__(self):
        super().__init__('rgbd_point_cloud_generator')

        self.color_sub = self.create_subscription(Image, 'mask', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)

        self.pcd_publisher = self.create_publisher(PointCloud2, '/filtered_point_cloud', 10)
        self.pub_centroid = self.create_publisher(PointCloud2,'/centroid',10)
        
        self.bridge = CvBridge()
        self.latest_color_image = None
        self.latest_depth_image = None

        # Camera Intrinsics
        hfov_radians = np.deg2rad(84) #1.57 # Horizontal field of view in radians
        image_width = 1280 # Image width in pixels
        image_height = 720 # Image height in pixels
        focal_length_px = image_width / (2 * np.tan(hfov_radians / 2))
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = image_width, 
            height = image_height, 
            fx = focal_length_px,  
            fy = focal_length_px, 
            cx = image_width / 2, 
            cy = image_height / 2
        )

    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg)
        self.latest_color_image = cv2.cvtColor(self.latest_color_image, cv2.COLOR_BGR2RGB)
    
    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.generate_point_cloud()

    def generate_point_cloud(self):
        if self.latest_color_image is None or self.latest_depth_image is None: 
            return
        mask = (self.latest_color_image.sum(axis=2) > 0).astype(np.uint8) * 255
        self.masked_depth_image = cv2.bitwise_and(self.latest_depth_image, self.latest_depth_image, mask=mask)
        # Convert image to open3d format
        depth_o3d = o3d.geometry.Image(self.masked_depth_image.astype(np.uint16))
        # Convert color image to Open3D format
        color_temp = cv2.cvtColor(self.latest_color_image, cv2.COLOR_RGB2BGR)
        # color_temp = self.latest_color_image
        color_o3d = o3d.geometry.Image(color_temp)

        # Create RGB-D image from color and depth
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d,
            depth_o3d, 
            depth_scale = 1000.0, 
            depth_trunc = 0.75,  #Originally 3
            convert_rgb_to_intensity=False
        )

        # Generate point cloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            self.intrinsic
        )

        # Convert Open3D Point Cloud to ROS PointCloud2
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) * 255 # Denormalize colors
        #points_colors = np.hstack((points, colors))
        rgb = colors[:,0] + colors[:,1] * 256 + colors[:,2] * 256 * 256
        # rgb = colors[:,0] + colors[:,1] + colors[:,2] #* 256 #* 256
        rgb = np.asarray(rgb, dtype=np.float32)
        points_rgb = np.hstack((points, rgb.reshape(-1,1)))

        # print(points_rgb.shape[0])

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_color_optical_frame"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        cloud_msg = pc2.create_cloud(header, fields, points_rgb)
        self.pcd_publisher.publish(cloud_msg)
        numpy_pc = pc2.read_points_numpy(cloud_msg)
        centroid = self.calculate_centroid(numpy_pc)
        centroid_msg = pc2.create_cloud_xyz32(header, [centroid])
        self.pub_centroid.publish(centroid_msg)
        self.get_logger().info("Centroid estimated at x:{}, y:{}, z:{}".format(centroid[0],centroid[1],centroid[2]))

    def calculate_centroid(self,points):
        x = points[:,0]
        y = points[:,1]
        z = points[:,2]
        return [np.mean(x), np.mean(y), np.mean(z)]

def main(args=None):
    rclpy.init(args=args)
    node = RGBDPointCloudGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()