import rclpy  #Importing the ros client library for python
from rclpy.node import Node #Importing the Node class for creating ROS2 Nodes
from std_msgs.msg import Int32 #Importing the standard message type 32-bit integer

#Creating a node number publisher
class NumberPublisher(Node):

    def __init__(self):
        #Creates a node names number_publisher
        super().__init__('number_publisher')
        """
        Creating a publisher of 
        Topic : number
        Message data type: Int32
        Queue Size : 10"""
        self.publisher_ = self.create_publisher(Int32, 'number', 10)

        self.get_logger().info("Enter number to publish :")

        #Creating an infinite loop to run while ROS2 is operating properly
        while rclpy.ok():
            #Try Except code block to handle error cases
            try:
                #Taking user input for a number and publishing the data
                num = int(input("Enter a number: "))
                self.publish_number(num)

            #If value entered is not an integer
            except ValueError:
                self.get_logger().error("Invalid Input! Enter an integer.")
            except KeyboardInterrupt:
                self.get_logger().info("Shutting Down...")
                break                


    def publish_number(self, num):
        #Creating a Int32 message
        msg = Int32()
        #Assigning data to the message as self.number
        msg.data = num
        #Using the publisher to per say publish the message
        self.publisher_.publish(msg)
        #Display the published number
        self.get_logger().info(f"Publishing: {msg.data}")
        

def main(args = None):
    #Starting ROS2 communication
    rclpy.init(args=args)
    #Initialize the node
    number_publisher = NumberPublisher()
    #Spin keeps the node alive
    rclpy.spin(number_publisher)

    #Clean up resources once node is closed
    number_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
