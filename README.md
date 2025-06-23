# ROS Notes:

## Learnings Summarized:
During the course of this week i have explored the practical and foundational concepts behind operating and creating nodes and how different components of ROS
work together.
1. Learnt how to operate ros using various command line tools
2. Learnt how to access/operate/create nodes, topics, messages
3. Learnt how to use the rclpy library with python and create a publisher and subscriber node in python
4. Learnt how to use the rclcpp library in cpp and create a publisher and subscriber node in cpp
5. Better understanding behind the code structure behind publisher and subscriber scripts. Both implementations helped me to understand how conecpts like topics, message passing, communication bw nodes play a crucial role behind the logic of a publisher and a subscriber.
6. Getting familiarized with RQT and its different use cases such as RQT Graph, RQT Logger and RQT Service caller using turtlesim package.
7. Built a solid foundation in ROS tools and node architecture.
8. Gained confidence in navigating, visualizing, and debugging ROS systems using both command-line and GUI tools.
- #### Week2:
9. Learnt about the communications systems and protocols that operate behind the scenes to allow data to be exchanged between publishers and subscribers.
10. Learnt about DDS, RTPS, UDPROS, Peer to Peer discovery.
11. Learnt how to setup launch files for nodes in package for cpp and python.
12. Learn how to add QoS profiles to nodes in cpp and python

- Running the nodes:
    ```bash
    cd ros_ws
    colcon build --packages-select number_square_py
    source install/setup.zsh
    ros2 launch number_square.launch.py
    ```
    ```bash
    cd ros_ws
    colcon build --packages-select number_square_py
    source install/setup.zsh
    ros2 launch number_square.launch.xml
    ```

### Week3 Task: Simulate a robot in gazebo
- Package name : robo_cpp
- How to Run
    ```bash
    cd ros_ws
    colcon build --packages-select robo_cpp
    source install/setup.zsh
    # Launching Gazebo and RViz
    ros2 launch robo_cpp spawn.launch.py

    #Launching teleop node
    ros2 launch robo_cpp teleop.launch.py

    #Running the lidar subscriber that avoids the obstacle in front of it
    ros2 run robo_cpp lidar subscriber 

    #Running an auto-navigator to navigate the robot through the maze
    ros2 run robo_cpp navigate_node
    ```

## Directory:
```
.
├── number_square_cpp
│   ├── CMakeLists.txt
│   ├── include
│   │   └── number_square_cpp
│   ├── launch
│   │   └── number_square.launch.xml
│   ├── package.xml
│   └── src
│       ├── number_publisher.cpp
│       └── square_subscriber.cpp
├── number_square_py
│   ├── launch
│   │   └── number_square.launch.py
│   ├── number_square_py
│   │   ├── __init__.py
│   │   ├── number_publisher.py
│   │   └── square_subscriber.py
│   ├── package.xml
│   ├── resource
│   │   └── number_square_py
│   ├── setup.cfg
│   ├── setup.py
│   └── test
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
└── robo_cpp
    ├── CMakeLists.txt
    ├── include
    │   └── robo_cpp
    ├── launch
    │   ├── spawn.launch.py
    │   └── teleop.launch.py
    ├── package.xml
    ├── rviz
    │   └── view_config.rviz
    ├── src
    │   └── lidar_subscriber.cpp
    ├── urdf
    │   └── robo_cpp.urdf.xacro
    └── worlds
        └── wall_world.world


```

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

## Launching Nodes using a launch file:
- The setup: Create a launch folder inside the package within it create a .py file for a node in python and .xml for a node in cpp.
- Naming of the file: THe file is name usually indicating the task it does or the nodes it runs such as two_nodes.launch.py or number_square.launch.xml

- #### Launch file code for number_square_cpp
```xml
<launch>
    <!-- >Launching number_publisher<!-->
    <node pkg = "number_square_cpp" exec="number_publisher" name="number_publisher" output="screen"/>
    <!-->Launching square_subscriber<!-->
    <node pkg = "number_square_cpp" exec="square_subscriber" name="square_subscriber" output="screen"/>
</launch>
```
![](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screenshot%20from%202025-06-01%2000-14-34.png)

- #### Launch file code for number_square_py
```py
# Importing necessary dependencies for setup launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Launch for number_publisher
        Node(
            package="number_square_py", # Package name
            executable = "number_publisher", # Executable name
            name = "number_publisher", # Node name
            output = "screen" # Tells console output to be printed onto the terminal
        ),

        #Launch for number subscriber
        Node(
            package="number_square_py",
            executable = "square_subscriber",
            name = "square_subscriber",
            output = "screen"
        )

    ])
```
![](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screenshot%20from%202025-06-01%2000-15-39.png)


- The Launch file must be specfied to be installed while building the package into the `setup.py` and `CMakeLists.txt` accordingly and
run using 
```bash
ros2 launch number_square_cpp launch number_square.launch.xml
ros2 launch number_square_py launch number_square.launch.py
```

## Setting up QoS profile:
- Code in python
```py
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE, #Ensures messages are recieved, resends messages if not recieved
            durability = DurabilityPolicy.VOLATILE,  #Doesn't store data for future nodes
            depth = 10 #Size of history storage incase tranferspeed is low
        )

self.publisher_ = self.create_publisher(Int32, 'number', qos_profile)        
```

- Code in CPP
```cpp
//Setting up a QoS profile
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
            .reliable() //Ensures message is recieved retransmits if not
            .durability_volatile() //Future nodes can't access already published data
            .keep_last(10); // Stores 10 messages

```
-**NOTE:** User input doesn't work when nodes are launched via a launch file hence for that to work the nodes have been updated so that they follow a timer based iterator that publishes data instead of a user input model.

## Week3 Learnings:

# 📝 Learning Summary: ROS 2 Mobile Robot with Gazebo Simulation

##  1. Robot Modeling with URDF & XACRO
- Built a **URDF robot model** with:
  - Rectangular chassis (base link)
  - Four wheels for differential drive
  - Joints for left and right wheels for Gazebo simulation
- Used **XACRO macros** for reusable wheel definitions
- Defined **inertial, visual, and collision properties**
- Added **ground clearance** and optimized geometry

##  2. Gazebo Plugins Integration
- Integrated `gazebo_ros_diff_drive` plugin:
  - Connected to rear wheels
  - Configured `wheel_radius`, `wheel_separation`, `max_wheel_torque`, etc.
  - Controlled via `/cmd_vel`, received odometry on `/odom`
- Added **camera** using `gazebo_ros_camera`:
  - Configured resolution, FOV, and remapped image topic
- Added **LIDAR** using `gazebo_ros_ray_sensor`:
  - 360° scan simulation
  - Published `sensor_msgs/LaserScan`

##  3. Autonomous Behavior with Obstacle Avoidance Node
- Created a C++ node that:
  - Subscribes to LIDAR data
  - Detects obstacles at front (e.g., index 180)
  - Publishes `geometry_msgs::Twist` to `/cmd_vel`
  - Overrides teleop when obstacles are detected

##  4. Creating a Custom World
- Built a custom **Gazebo world**
- Spawned robot at specific location
- Added **walls/obstacles** for navigation testing
- Observed realistic physical interactions

##  5. Visualization in RViz2
- Configured **RViz2** to display:
  - `LaserScan`, `Camera`, `TF`, `RobotModel`
- Saved and reused `.rviz` config
- Used `robot_state_publisher` and `joint_state_publisher`

##  Key Concepts & Skills Learned

| Topic                  | Learning Outcome                                           |
|-----------------------|------------------------------------------------------------|
| URDF/XACRO            | Modular robot modeling with macros                         |
| Gazebo Plugins        | Simulated diff drive, camera, and lidar                    |
| ROS 2 Nodes           | Processed sensor data and published movement commands      |
| Obstacle Avoidance    | Reactive behavior using LIDAR and Twist messages           |
| RViz2 Configuration   | Real-time robot visualization setup                        |
| Robot Simulation      | Integrated sensor-actuator loop in simulated environment   |

##  Tools & Packages Used
- `gazebo_ros_diff_drive`
- `gazebo_ros_camera`, `gazebo_ros_ray_sensor`
- `robot_state_publisher`, `joint_state_publisher`
- `teleop_twist_keyboard`
- `rviz2`, `xacro`, `rclcpp`, `sensor_msgs`, `geometry_msgs`

![](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screenshot%20from%202025-06-20%2022-59-05.png)
![](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screenshot%20from%202025-06-20%2022-59-16.png)
![](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/obstacleavoidance.gif)

## Video Links:
- [Obstacle Avoidance](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screencast%20from%2006-21-2025%2010%3A42%3A12%20AM.webm)
- [Teleop Maneuvering](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/Screencast%20from%2006-21-2025%2010%3A52%3A08%20AM.webm)
- [Auto Navigation](https://github.com/ArpitSrivastavaIN/ros2_notes/blob/main/images/navigation_vid.mp4)