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
		-  
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