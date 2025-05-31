#include "rclcpp/rclcpp.hpp" // Importing the ros2 c++ client library
#include "std_msgs/msg/int32.hpp" // Importing the integer message type

//Creating a node class SquareSubscriber
class SquareSubscriber : public rclcpp::Node {
public:
    //Constructing the node named square_subscriber
    SquareSubscriber() : Node("square_subscriber") {
        
        //Setting up a QoS profile
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
            .reliable() //Ensures message is recieved retransmits if not
            .durability_volatile() //Future nodes can't access already published data
            .keep_last(10); // Stores 10 messages

        //Creating a subscription on topic 'number' message type Int32 and a queue size of 10
        subscription_ = this -> create_subscription<std_msgs::msg::Int32>(
            "number", qos,
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
