import rclpy #Importing the ros client library for python
from rclpy.node import Node # Importing Node class from rclpy
from std_msgs.msg import Int32 #Importing the standard message type 32-bit integer
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

#Creaitng a node named SquareSubscriber 
class SquareSubscriber(Node):

    def __init__(self):
        # Creates a node named square_subscriber
        super().__init__('square_subscriber')
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE, 
            durability = DurabilityPolicy.VOLATILE,
            depth = 10
        )
        """
        Creating a subscription of
        Topic : number
        message type : Int32
        queue size : 10"""
        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            qos_profile
        )
        self.subscription # Prevent unused variable warning

    # Function to square the published data and log that info
    def listener_callback(self, msg):
        square = msg.data ** 2
        self.get_logger().info(f"Recieved: {msg.data}, Square : {square}")

def main(args = None):
    #Initializing the ros library
    rclpy.init(args=args)
    #Creating the subscriber
    square_subscriber = SquareSubscriber()
    #Spin the node to keep it alive
    rclpy.spin(square_subscriber)

    #Clean up resources once node is closed
    square_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
