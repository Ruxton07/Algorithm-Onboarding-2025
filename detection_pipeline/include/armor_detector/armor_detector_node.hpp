#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>

class CameraSubscriberNode : public rclcpp::Node {
    public:
        CameraSubscriberNode();
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};
