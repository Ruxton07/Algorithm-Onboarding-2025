#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

CameraSubscriberNode::CameraSubscriberNode()
: Node("camera_subscriber_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(

        
}

// call back to read from topic, that will give you your matrix
void CameraSubscriberNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    RCLCPP_INFO(this->get_logger(), "i got an image with size: %dx%d", frame.cols, frame.rows);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}