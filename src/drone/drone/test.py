import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class ArmDrone(Node):
    def __init__(self):
        super().__init__('arm_drone_node')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')

    def arm(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        self.get_logger().info("Sent arm request, waiting for response...")
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            result = future.result()
            if result.success:
                self.get_logger().info("Drone armed successfully!")
            else:
                self.get_logger().error("Failed to arm the drone.")
        else:
            self.get_logger().error("Future not completed. Timeout?")

def main(args=None):
    rclpy.init(args=args)
    node = ArmDrone()
    node.arm()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
