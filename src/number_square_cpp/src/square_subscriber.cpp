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
