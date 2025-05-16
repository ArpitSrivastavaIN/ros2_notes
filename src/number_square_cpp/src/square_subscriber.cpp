#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SquareSubscriber : public rclcpp::Node {
public:
    SquareSubscriber() : Node("square_subscrober") {
        subscription_ = this -> create_subscription<std_msgs::msg::Int32>(
            "number", 10,
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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareSubscriber>());
    rclcpp::shutdown();
    return 0;
}
