#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/front_camera/image_raw", 10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

        cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "Orange Box Detector started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            if (frame.empty()) return;

            // Convert to HSV color space
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

            // Orange HSV range
            cv::Scalar lower_orange(10, 100, 100);
            cv::Scalar upper_orange(25, 255, 255);
            cv::Mat mask;
            cv::inRange(hsv, lower_orange, upper_orange, mask);

            // Morphological operations to remove noise
            cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto &cnt : contours)
            {
                if (cv::contourArea(cnt) < 500) continue;

                cv::Rect box = cv::boundingRect(cnt);
                box &= cv::Rect(0, 0, frame.cols, frame.rows); // clip to frame size
                cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, "Orange Box", cv::Point(box.x, box.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }

            cv::imshow("Camera Feed", frame);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
