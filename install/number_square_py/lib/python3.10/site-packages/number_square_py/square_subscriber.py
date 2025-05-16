import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SquareSubscriber(Node):
    def __init__(self):
        super().__init__('square_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        square = msg.data ** 2
        self.get_logger().info(f"Recieved: {msg.data}, Square : {square}")

def main(args = None):
    rclpy.init(args=args)
    square_subscriber = SquareSubscriber()
    rclpy.spin(square_subscriber)

    square_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
