import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveCartesian
from sensor_msgs.msg import PointCloud2 
from sensor_msgs_py import point_cloud2 as pc2
from xarm_msgs.srv import SetInt16ById, SetInt16

class move_line(Node):
    def __init__(self):
        super().__init__('Move_line')
        self.get_logger().info('Move_line initialized')

        # Create variables
        self.centroid_recieved = False
        self.x = 0
        self.y = 0
        self.z = 0
        self.home = [250.0, 0.0, 400.0, 3.14, 0.0, 0.0]
        self.pose = self.home
        
        # Define constants
        self.req = MoveCartesian.Request()

        # Create subscribers
        self.sub_centroid = self.create_subscription(PointCloud2,'/centroid_world', self.sub_centroid_callback, 10)
        
        # Initiation routine 
        self.init_robot()
        self.get_logger().info('Robot initialized')
        
        # Move to image caṕture position
        self.send_request(pose=self.home)
        
        #  Create timmer
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def init_robot(self):
        # Create a client for the /ufactory/motion_enable service
        self.motion_enable_client = self.create_client(SetInt16ById, '/ufactory/motion_enable')
        while not self.motion_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ufactory/motion_enable not available, waiting...')

        # Call the /ufactory/motion_enable service
        motion_enable_request = SetInt16ById.Request()
        motion_enable_request.id = 8
        motion_enable_request.data = 1
        future = self.motion_enable_client.call_async(motion_enable_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Motion enable successful')
        else:
            self.get_logger().error('Failed to call /ufactory/motion_enable')

        # Create a client for the /ufactory/set_mode service
        self.set_mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ufactory/set_mode not available, waiting...')

        # Call the /ufactory/set_mode service
        set_mode_request = SetInt16.Request()
        set_mode_request.data = 0
        future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Set mode successful')
        else:
            self.get_logger().error('Failed to call /ufactory/set_mode')

        # Create a client for the /ufactory/set_state service
        self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ufactory/set_state not available, waiting...')

        # Call the /ufactory/set_state service
        set_state_request = SetInt16.Request()
        set_state_request.data = 0
        future = self.set_state_client.call_async(set_state_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Set state successful')
        else:
            self.get_logger().error('Failed to call /ufactory/set_state')

        # Create a client for the /ufactory/set_position service
        self.client = self.create_client(MoveCartesian, '/ufactory/set_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Ufactory service available')

    def send_request(self, pose = None):
        if pose is None:
            self.req.pose = self.pose
        else: 
            self.req.pose = pose
        self.future = self.client.call_async(self.req)
        self.get_logger().info('Request sent')
        # rclpy.spin_until_future_complete(self, self.future)
        # self.get_logger().info("Service call completed")  

    def sub_centroid_callback(self, msg):
        target = pc2.read_points_numpy(msg)
        self.x = float(target[0][0] * 1000)
        self.y = float(target[0][1] * 1000)
        self.z = float(target[0][2] * 1000 + 40)
        self.get_logger().info("Centroid received x: {}, y: {}, z: {}".format(self.x, self.y, self.z),once=True)
        self.centroid_recieved = True

    def grasp_routine(self, target):
        self.send_request(pose=target)
        ans = input("Do you want to grasp the object? (y/n)")
        ans = input('DO you want to move to home position? (y/n)')
        if ans == 'y':
            self.send_request(pose=self.home)
            self.get_logger().info("Moving to home position",once=True)

    def timer_callback(self):
        if not self.centroid_recieved: return
        self.get_logger().info("Centroid estimated to \n x: {}\n y: {}\n z: {}".format(self.x, self.y, self.z))
        answer = input("Do you want to move to this position? (y/n)")
        if answer == 'y':
            self.pose = [self.x, self.y, self.z, 3.14, 0.0, 0.0]
            self.grasp_routine(self.pose)
        else:
            self.get_logger().info("Position not accepted",once=True)
def main(args=None):
    # Required lines for any node
    rclpy.init(args=args)
    node = move_line()
    rclpy.spin(node)
    # Optional but good practices
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()