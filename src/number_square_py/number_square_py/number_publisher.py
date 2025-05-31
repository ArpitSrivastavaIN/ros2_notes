import rclpy  #Importing the ros client library for python
from rclpy.node import Node #Importing the Node class for creating ROS2 Nodes
from std_msgs.msg import Int32 #Importing the standard message type 32-bit integer
#Importing required dependencies for a QoS profile
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

#Creating a node number publisher
class NumberPublisher(Node):

    def __init__(self):
        #Creates a node names number_publisher
        super().__init__('number_publisher')
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.VOLATILE, 
            depth = 10
        )

        """
        Creating a publisher of 
        Topic : number
        Message data type: Int32
        Queue Size : 10"""
        self.publisher_ = self.create_publisher(Int32, 'number', qos_profile)

        self.get_logger().info("Starting automatic number publishing.")

        self.count = 0  # Initialize the counter

        # Create a timer to publish number every 1 second (1000 milliseconds)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        #Publish numbers in increasing order automatically
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.count += 1  # Increment the counter

def main(args=None):
    #Starting ROS2 communication
    rclpy.init(args=args)
    #Initialize the node
    number_publisher = NumberPublisher()
    #Spin keeps the node alive and processing timers/callbacks
    rclpy.spin(number_publisher)

    #Clean up resources once node is closed
    number_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

