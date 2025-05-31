#include "rclcpp/rclcpp.hpp" //Importing ros2 c++ client library
#include "std_msgs/msg/int32.hpp" //Importing ros2 standard integer message type
#include <iostream> //Importing c++ input/output library

using namespace std::chrono_literals;

//Creating NumberPublisher class inheriting from Node class
class NumberPublisher : public rclcpp::Node {
public:
    //Constructing the Node with name "name_publisher"
    NumberPublisher() : Node("number_publisher"), count_(0) {

        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
        .reliable()
        .durability_volatile()
        .keep_last(10);

        //Creating a publisher that publishes to topic 'number using Int32 and has a queue size of 10
        publisher_ = this -> create_publisher<std_msgs::msg::Int32>("number", qos);
        //Creates a timer that triggers every second
        timer_ = this->create_wall_timer(
            1000ms, // 1 second interval
            std::bind(&NumberPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Number publisher started with timer...");
    }

private:
    void timer_callback() {
        //Creating a message and assigning count_ to it
        auto message = std_msgs::msg::Int32();
        message.data = count_++;
        
        // Logging and Publishing the number
        RCLCPP_INFO(this->get_logger(), "Published: %d", message.data);
        publisher_->publish(message);
    }
    
    int count_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
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
