cmake_minimum_required(VERSION 3.8)
project(robo_cpp)

# Compiler settings for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

# Core build system
find_package(ament_cmake REQUIRED)

# ROS 2 and message dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)



# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------

# Lidar obstacle avoidance node
add_executable(lidar_subscriber src/lidar_subscriber.cpp)
ament_target_dependencies(lidar_subscriber rclcpp sensor_msgs geometry_msgs)

#Navigation Node
add_executable(navigate_node src/navigate_node.cpp)
ament_target_dependencies(navigate_node rclcpp sensor_msgs geometry_msgs)

#Image Subscriber Node
add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge OpenCV ament_index_cpp)

# Install executable
install(TARGETS
  lidar_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  navigate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)



# ------------------------------------------------------------------------------
# Install Resources: Launch files, URDFs, Worlds, RViz config
# ------------------------------------------------------------------------------

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# URDF/Xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Gazebo world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# RViz configuration
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Linting and Tests
# ------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright and cpplint if not ready
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Package Declaration
# ------------------------------------------------------------------------------

ament_package()
