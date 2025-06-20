#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class LidarMover : public rclcpp::Node
{
public:
    LidarMover() : Node("lidar_mover"), moving_(false), front_distance_(std::numeric_limits<float>::infinity())
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/gazebo_ros_laser/out", 10,
            std::bind(&LidarMover::scan_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&LidarMover::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "LidarMover node started.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.size() > 180) {
            float r = msg->ranges[180];
            if (r >= 0.3 && std::isfinite(r)) {
                front_distance_ = r;
            }
        }
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        if (moving_ && front_distance_ < 0.6) {
            moving_ = false;
            RCLCPP_INFO(this->get_logger(), "Obstacle at %.2fm: STOP", front_distance_);
        } else if (!moving_ && front_distance_ > 0.7) {
            moving_ = true;
            RCLCPP_INFO(this->get_logger(), "Path clear at %.2fm: MOVING FORWARD", front_distance_);
        }

        cmd.linear.x = moving_ ? 0.2 : 0.0;
        publisher_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool moving_;
    float front_distance_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMover>());
    rclcpp::shutdown();
    return 0;
}
