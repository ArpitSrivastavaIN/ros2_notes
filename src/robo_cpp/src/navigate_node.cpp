#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <limits>
#include <cmath>

class NavigateNode : public rclcpp::Node
{
public:
    NavigateNode() : Node("navigate_node"),
                     moving_(false),
                     rotating_(false),
                     rotation_started_(false),
                     rotate_left_(true),
                     front_distance_(std::numeric_limits<float>::infinity())
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/gazebo_ros_laser/out", 10,
            std::bind(&NavigateNode::laser_parse, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NavigateNode::control_loop, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool moving_;
    bool rotating_;
    bool rotation_started_;
    bool rotate_left_;

    float front_distance_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    rclcpp::Time start_time_;

    void laser_parse(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;

        if (msg->ranges.size() > 180)
        {
            float r = msg->ranges[180];
            if (r >= 0.3 && std::isfinite(r))
            {
                front_distance_ = r;
            }
        }
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        float left = last_scan_->ranges[270];
        float right = last_scan_->ranges[90];
        float diff = std::abs(left - right);

        if (diff < 0.5 && front_distance_ < 1.05 && front_distance_ > 0.95)
        {
            cmd.linear.x = 0.0;
            publisher_->publish(cmd);
    
            RCLCPP_WARN(this->get_logger(), "[SHUTDOWN] Left-Right diff %.2f too small. Shutting down...", diff);
            rclcpp::shutdown();
            return;
        }

        // Stop and prepare to rotate
        if (moving_ && front_distance_ < 1.05 && front_distance_ > 0.95)
        {
            moving_ = false;
            rotating_ = true;
            rotation_started_ = false;
            RCLCPP_INFO(this->get_logger(), "[STOP] : Obstacle at %.3fm", front_distance_);
        }

        if (rotating_)
        {
            // Set rotation direction and timer only once
            if (!rotation_started_)
            {

                RCLCPP_INFO(this->get_logger(), "[ROTATING] : Left  : %.3fm.", left);
                RCLCPP_INFO(this->get_logger(), "[ROTATING] : Right : %.3fm.", right);

                rotate_left_ = (left > right);
                start_time_ = this->now();
                rotation_started_ = true;

                RCLCPP_INFO(this->get_logger(), "[ROTATING] Direction: %s", rotate_left_ ? "Left" : "Right");
            }

            // Check rotation duration
            auto elapsed = this->now() - start_time_;
            if (elapsed.seconds() < 5.45)
            {
                cmd.angular.z = rotate_left_ ? 0.3 : -0.3;
            }
            else
            {
                cmd.angular.z = 0.0;
                rotating_ = false;
                rotation_started_ = false;
                RCLCPP_INFO(this->get_logger(), "[ROTATING] : Rotation Completed.");
            }
        }

        // Start moving again
        if (!moving_ && !rotating_)
        {
            moving_ = true;
            RCLCPP_INFO(this->get_logger(), "[MOVING] : Moving Forward.");
        }

        cmd.linear.x = moving_ ? 0.3 : 0.0;
        publisher_->publish(cmd);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateNode>());
    rclcpp::shutdown();
    return 0;
}
