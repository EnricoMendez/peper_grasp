import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import std_msgs.msg
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo 
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
import cv2

class RGBDPointCloudGenerator(Node):
    def __init__(self):
        super().__init__('rgbd_point_cloud_generator')

        self.color_sub = self.create_subscription(Image, 'mask', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo,'/camera/aligned_depth_to_color/camera_info', self.info_callback, 10)

        self.pcd_publisher = self.create_publisher(PointCloud2, '/filtered_point_cloud', 10)
        self.pub_centroid = self.create_publisher(PointCloud2, '/centroid', 10)
        self.pub_centroid_world = self.create_publisher(PointCloud2, '/centroid_world', 10)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.bridge = CvBridge()
        self.latest_color_image = None
        self.latest_depth_image = None
        self.color_recieved = False
        self.depth_recieved = False
        self.intrinsic = None
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = 1280,  # Image width in pixels
            height = 720,  # Image height in pixels

            fx = 922.1322631835938,
            fy = 922.2659301757812,
            cx = 654.000244140625,
            cy = 361.27093505859375,)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def info_callback(self, msg):
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width = msg.width, 
            height = msg.height, 
            fx = msg.k[0],  
            fy = msg.k[4], 
            cx = msg.k[2], 
            cy = msg.k[5]
        )

    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg)
        # self.latest_color_image = cv2.cvtColor(self.latest_color_image, cv2.COLOR_BGR2RGB)
    
    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def timer_callback(self):
        if self.latest_color_image is None or self.latest_depth_image is None or self.intrinsic is None: 
            self.get_logger().warn("Messages not recieved yet.")
            return
        self.get_logger().info("Messages recieved. Generating point cloud.", once=True)
        _, img_binary = cv2.threshold(self.latest_color_image, 1, 255, cv2.THRESH_BINARY)
        img_binary = cv2.cvtColor(img_binary, cv2.COLOR_BGR2GRAY)
        num_blobs, blobs = cv2.connectedComponents(img_binary)
        centroids = []
        for i in range (1,num_blobs):
            mask = (blobs == i).astype(np.uint8)
            masked_depth_image = cv2.bitwise_and(self.latest_depth_image, self.latest_depth_image, mask=mask)
            pc = self.generate_point_cloud(masked_depth_image)
            cloud_msg = self.generate_pc2_msg(pc, "camera_color_optical_frame")
            numpy_pc = pc2.read_points_numpy(cloud_msg)
            centroid = self.calculate_centroid(numpy_pc)
            centroids.append(centroid)
        centroid_msg = self.np2ros(np.array(centroids), 'camera_color_optical_frame')
        self.pub_centroid.publish(centroid_msg)
        self.get_logger().info("Centroid published successfully", once=True)
        world_centroid = self.transform_point(centroids, 'world')
        if world_centroid is not None:
            world_centroid_msg = self.np2ros(np.array([world_centroid]), 'world')
            self.pub_centroid_world.publish(world_centroid_msg)
            self.get_logger().info("World centroid published successfully", once=True)
        

    def generate_point_cloud(self, masked_depth_image):
        # Convert image to open3d format
        depth_o3d = o3d.geometry.Image(masked_depth_image.astype(np.uint16))
        # Convert color image to Open3D format
        color_temp = self.latest_color_image
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
        # points_colors = np.hstack((points, colors))
        # rgb = colors[:,0] + colors[:,1] * 256 + colors[:,2] * 256 * 256
        rgb = colors[:,0] + colors[:,1] + colors[:,2] #* 256 #* 256
        rgb = np.asarray(rgb, dtype=np.float32)
        points_rgb = np.hstack((points, rgb.reshape(-1,1)))

        return points_rgb
    
    def generate_pc2_msg(self, points_rgb, frame_id):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        cloud_msg = pc2.create_cloud(header, fields, points_rgb)
        return cloud_msg

    def np2ros(self, points, frame_id):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        return cloud_msg

    def calculate_centroid(self, points):
        x = points[:,0]
        y = points[:,1]
        z = points[:,2]
        return [np.mean(x), np.mean(y), np.mean(z)]

    def transform_point(self, points, target_frame):
        try:
            # Look up the transformation at the current time
            transform = self.tf_buffer.lookup_transform(target_frame, 'camera_color_optical_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Extract translation and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # Convert ROS Quaternion to a rotation matrix
            rot_matrix = self.quaternion_to_rotation_matrix(rotation)

            # Apply rotation and translation to the point
            
            points_world = []
            for point in points:
                point = np.array(point)
                point_world = rot_matrix @ point + np.array([translation.x, translation.y, translation.z])
                points_world.append(point_world)
            point_world = rot_matrix @ point + np.array([translation.x, translation.y, translation.z])

            self.get_logger().info("Centroid estimated at x:{}, y:{}, z:{}".format(point_world[0],point_world[1],point_world[2]), once=True)
            return points_world
        
        except tf2_ros.LookupException:
            self.get_logger().error("Transform lookup failed.")
            return None
        except tf2_ros.ExtrapolationException:
            self.get_logger().error("Transform extrapolation failed.")
            return None

    def quaternion_to_rotation_matrix(self, q):
        # Convert a quaternion into a full three-dimensional rotation matrix.
        x, y, z, w = q.x, q.y, q.z, q.w
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        rot_matrix = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
        ])
        return rot_matrix

def main(args=None):
    rclpy.init(args=args)
    node = RGBDPointCloudGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
