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

            if(!std::cin){
                RCLCPP_WARN(this->get_logger(), "Invalid Value!");
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }
            
            auto message = std_msgs::msg::Int32();
            message.data = num;
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
