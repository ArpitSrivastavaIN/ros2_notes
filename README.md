# ROS Notes:
## CLI Tools:

-  Setting Up ROS2 commands:
	
	To run ros2 commands every time a new shell is being used setup.zsh has to be changed and at startup only ros gets setup
	echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
	source ~/.zshrc
	
- Running a package in ros:
	
	```console
    ros2 run <package_name> <executable_name>
	ros2 run turtlesim turtle_teleop_key
    ```
	
- Checking active nodes, topics, services
	 ```console
    ros2 node list
	ros2 topic list
	ros2 service list
	ros2 action list
    ```

- Using **RQT**:
	RQT is a GUI framework for ros that allows the user to create and delete new nodes, change parameters within nodes and has various plugins such as `rqt_graph` a graph that shows how the various nodes are connected. `rqt_plot` so as to view sensor data and `rqt_console` to display log messages. It has a service_caller which can be used to access various services.

    ![](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screenshot%20from%202025-05-17%2012-24-01.png)

- ### Ros Nodes:.
    Command to find out how many nodes are running .
	```bash
    arpit@sri ~ % ros2 node list
    /number_publisher
    /square_subscrober
    ```
    `ros2 node list` : to get the list of nodes

    `ros2 node info number_subscriber` : To get the information about a particular node


- ### Ros topics:
	- Getting the list of topics and their type
        ```console
        arpit@sri ~ % ros2 topic list -t
        /number [std_msgs/msg/Int32]
        /parameter_events [rcl_interfaces/msg/ParameterEvent]
        /rosout [rcl_interfaces/msg/Log]
        ```

	- Displaying what data is being sent via the topic /number
        ```console
        arpit@sri ~ % ros2 topic echo /number 
        data: 100
        data: 20
        data: 30
        ```
	- Displaying topic info
        ```console
        arpit@sri ~ % ros2 topic info /number
        Type: std_msgs/msg/Int32
        Publisher count: 1
        Subscription count: 1
        ```

	- Publishing a message on a topic :
        ```console
        arpit@sri ~ % ros2 topic pub /number std_msgs/msg/Int32 "{data: 20}"
        ```

## Creating a package 
- ### Publisher and Subscriber in CPP:
	- #### Setting up the package:
		- Creating the package :
			```bash
            ros2 pkg create --build-type ament_cmake number_square_cpp --dependencies rclcpp std_msgs
            ``` 
		- Changes in package.xml (gets done by the package creation command)

			```xml
            <depend>rclcpp</depend>
            <depend>std_msgs</depend>
            ```
			
		- Update **CmakeLists.txt**

            ```cmake
            # Add Executables
            add_executable(number_publisher src/number_publisher.cpp)
            add_executable(square_subscriber src/square_subscriber.cpp)

            # Link dependencies
            ament_target_dependencies(number_publisher rclcpp std_msgs)
            ament_target_dependencies(square_subscriber rclcpp std_msgs)

            # Install executables
            install(TARGETS
            number_publisher
            square_subscriber
            DESTINATION lib/${PROJECT_NAME}
            )
            ```

		- Building the package

            ```bash
            colcon build --packages-select number_square_cpp
            ```
	- #### The code:
		- **number_publisher.cpp**:
            - Is a publisher node that takes user input of an integer an publishes it to the topic '\number'. It can also handle error cases where the user has entered a value that is not an integer.

            ```cpp
            #include "rclcpp/rclcpp.hpp" //Importing ros2 c++ client library
            #include "std_msgs/msg/int32.hpp" //Importing ros2 standard integer message type
            #include <iostream> //Importing c++ input/output library

            using namespace std::chrono_literals;

            //Creating NumberPublisher class inheriting from Node class
            class NumberPublisher : public rclcpp::Node {
            public:
                //Constructing the Node with name "name_publisher"
                NumberPublisher() : Node("number_publisher") {
                    //Creating a publisher that publishes to topic 'number using Int32 and has a queue size of 10
                    publisher_ = this -> create_publisher<std_msgs::msg::Int32>("number", 10);
                    //Creates a timer that triggers every second

                    //Taking user input for a number to be squared and publishing the number
                    std::cout << "Enter numbers to publish (press Ctrl+C to quit):\n";
                    
                    while (rclcpp::ok()) {
                        std::cout << "> ";
                        int num;
                        std::cin >> num;

                        //Check if the previous cin command execution failed 
                        if(!std::cin){
                            RCLCPP_WARN(this->get_logger(), "Invalid Value!");
                            // Clear out the error and move to the next iteration
                            std::cin.clear();
                            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            continue;
                        }
                        
                        //Creating a message and assigning num to it
                        auto message = std_msgs::msg::Int32();
                        message.data = num;
                        // Logging and Publishing the user input number
                        RCLCPP_INFO(this->get_logger(), "Published: %d", message.data);
                        publisher_->publish(message);
                    }
                }

            private:
                int count_;
                rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
                
            };


            int main(int argc, char *argv[]) {
                //Initializes ros2
                rclcpp::init(argc, argv);
                //Creating node instance
                rclcpp::spin(std::make_shared<NumberPublisher>());
                //Closing the node
                rclcpp::shutdown();
                return 0;
            }

            ```

        - **square_subscriber.cpp**:
            - Is a subscriber node that listens to the topic '\number' and logs the square of the number published. 

            ```cpp
            #include "rclcpp/rclcpp.hpp" // Importing the ros2 c++ client library
            #include "std_msgs/msg/int32.hpp" // Importing the integer message type

            //Creating a node class SquareSubscriber
            class SquareSubscriber : public rclcpp::Node {
            public:
                //Constructing the node named square_subscriber
                SquareSubscriber() : Node("square_subscriber") {
                    //Creating a subscription on topic 'number' message type Int32 and a queue size of 10
                    subscription_ = this -> create_subscription<std_msgs::msg::Int32>(
                        "number", 10,
                        //Lambda function to capture the message and log its square
                        [this](const std_msgs::msg::Int32::SharedPtr msg) {
                            int num = msg -> data;
                            int square = num * num;
                            RCLCPP_INFO(this->get_logger(), "Recieved: %d, Square: %d", num, square);
                        }
                    );
                }

            private:
                rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
            };

            int main(int argc, char *argv[]) {
                //Initializing the ros2 library
                rclcpp::init(argc, argv);
                //Spin makes the node SquareSubscriber stay alive
                //Using make_shared helps manage the lifecycle of the node effeciently
                rclcpp::spin(std::make_shared<SquareSubscriber>());
                //Shutting the node down
                rclcpp::shutdown();
                return 0;
            }

            ```

- ### Publisher and Subscriber in Python
	- #### Setting up the package:
		-  Creating the package:

            ```bash
            ros2 pkg create --build-type ament_python <pkg_name> --dependencies rclpy std_msgs
            ```
		-  Add the scrips to **setup.py**

            ```py
            entry_points={
                'console_scripts': [
                    'number_publisher = number_square_py.number_publisher:main',
                    'square_subscriber = number_square_py.square_subscriber:main',
                ],
            },
            ```
		- Building the package

            ```console
            colcon build --packages-select number_square_py
            ```
	- #### The code:
        - **number_publisher.py**:
            - Is a publisher node that takes user input of an integer an publishes it to the topic '\number'. It can also handle error cases where the user has entered a value that is not an integer.

            ```py
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

            ```

            ![](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screenshot%20from%202025-05-17%2016-51-30.png)

        - **square_subscriber.py**:
            - Is a subscriber node that listens to the topic '\number' and logs the square of the number published. 

            ```py
            import rclpy #Importing the ros client library for python
            from rclpy.node import Node # Importing Node class from rclpy
            from std_msgs.msg import Int32 #Importing the standard message type 32-bit integer

            #Creaitng a node named SquareSubscriber 
            class SquareSubscriber(Node):

                def __init__(self):
                    # Creates a node named square_subscriber
                    super().__init__('square_subscriber')
                    """
                    Creating a subscription of
                    Topic : number
                    message type : Int32
                    queue size : 10"""
                    self.subscription = self.create_subscription(
                        Int32,
                        'number',
                        self.listener_callback,
                        10
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

            ```

            ![](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screenshot%20from%202025-05-17%2016-52-07.png)