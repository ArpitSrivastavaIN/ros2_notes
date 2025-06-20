#include "rclcpp/rclcpp.hpp"                      // ROS 2 C++ client library
#include "sensor_msgs/msg/laser_scan.hpp"         // Message type for LIDAR scan
#include "geometry_msgs/msg/twist.hpp"            // Message type for velocity commands

class LidarMover : public rclcpp::Node
{
public:
    // Constructor: Initializes node, subscriptions, publisher, and timer
    LidarMover() : Node("lidar_mover"),
                   moving_(false),  // Robot initially stopped
                   front_distance_(std::numeric_limits<float>::infinity())  // No valid reading yet
    {
        // Subscribe to LIDAR scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/gazebo_ros_laser/out", 10,
            std::bind(&LidarMover::scan_callback, this, std::placeholders::_1));

        // Create publisher to publish velocity commands to /cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create a periodic timer (100ms) to run the control loop
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&LidarMover::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "LidarMover node started.");
    }

private:
    // Callback function for processing incoming LIDAR scan data
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Check if the 180th index (front) is within valid range
        if (msg->ranges.size() > 180) {
            float r = msg->ranges[180];

            // Accept only valid, finite distances above 0.3m
            if (r >= 0.3 && std::isfinite(r)) {
                front_distance_ = r;
            }
        }
    }

    // Main control loop that runs periodically
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;  // Message to hold velocity command

        // If currently moving and obstacle detected within 0.6m → STOP
        if (moving_ && front_distance_ < 0.6) {
            moving_ = false;
            RCLCPP_INFO(this->get_logger(), "Obstacle at %.2fm: STOP", front_distance_);
        }
        // If currently stopped and clear space ahead beyond 0.7m → MOVE
        else if (!moving_ && front_distance_ > 0.7) {
            moving_ = true;
            RCLCPP_INFO(this->get_logger(), "Path clear at %.2fm: MOVING FORWARD", front_distance_);
        }

        // Set movement command: move forward if `moving_` is true, else stop
        cmd.linear.x = moving_ ? 0.2 : 0.0;

        // Publish the movement command
        publisher_->publish(cmd);
    }

    // ROS 2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_; // LIDAR data subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;         // Velocity command publisher
    rclcpp::TimerBase::SharedPtr timer_;                                        // Timer for periodic execution

    // Internal state variables
    bool moving_;            // Tracks whether robot is currently moving
    float front_distance_;   // Most recent distance reading at front
};

// Main function: initialize, spin node, and shutdown
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                 // Initialize ROS 2
    rclcpp::spin(std::make_shared<LidarMover>());             // Run the node
    rclcpp::shutdown();                                       // Shutdown ROS 2
    return 0;
}
