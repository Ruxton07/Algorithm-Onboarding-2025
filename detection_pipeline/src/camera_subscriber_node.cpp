#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

CameraSubscriberNode::CameraSubscriberNode()
: Node("camera_subscriber_node")
{
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::SensorDataQoS);

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}