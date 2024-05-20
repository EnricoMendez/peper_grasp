import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveCartesian

class move_line(Node):
    def __init__(self):
        super().__init__('Move_line')
        self.get_logger().info('Move_line initialized')

        # Create variables
        
        # Define constants
        
        # Create client
        self.client = self.create_client(MoveCartesian, '/ufactory/set_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveCartesian.Request()
        self.req.pose = [250.0, 0.0, 250.0, 3.14, 0.0, 0.0]
    
    def send_request(self):
        self.future = self.client.call_async(self.req)
        self.get_logger().info('Request sent')
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Service call completed")


        
        
def main(args=None):
    # Required lines for any node
    rclpy.init(args=args)
    node = move_line()
    node.send_request()
    rclpy.spin(node)
    # Optional but good practices
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()