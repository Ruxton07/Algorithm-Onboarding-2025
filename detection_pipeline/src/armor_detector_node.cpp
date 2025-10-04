/*
 *  READ ALL COMMENTS IN THIS FILE TO UNDERSTAND THE CODE THAT HAS BEEN WRITTEN FOR YOU
 *
 *  This file will create a subscriber node that reads images from the topic where camera_publisher_node publishes
 *  images. It will contain the functionality necessary for locating armor plates within a given image.
 *
 *  Since this file implements a ROS node like camera_publisher_node does, you may want to refer to that file to get
 *  ideas for things you need to consider while writing your code.
 */

#include "../include/armor_detector/armor_detector_node.hpp"

/*
 *  This is the constructor for our subscriber node. It initializes the node inherited from the base class and creates
 *  the subscription to the topic with messages from camera_publisher_node.
 */
ArmorDetectorNode::ArmorDetectorNode() : Node("armor_detector_node"), frame_count(0)
{
    // Subscribe to the camera publisher topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode subscribed to topic");
}

/*
 *  This is an image callback method. It fetches messages (which are images in this case) from the topic this node
 *  subscribes to. The method will also run your armor detection algorithm on the image and show the result.
 *
 *  Callback methods are how we actually read data from a topic. Notice the parameter type and compare it to that of
 *  image_sub_. Our subscription here fetches images, and this method is how you actually operate on that image. Even
 *  though your image processing logic is in a different method, this callback uses those methods as helpers. Make sure
 *  that you understand how callbacks function in a ROS node architecture. If you completed the constructor correctly,
 *  you should know how a callback connects to a subscription in code.
 */
void ArmorDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Images are represented by cv::Mat objects. This will be useful when you write image processing logic.
    cv::Mat frame;
    
    // Read the image from the topic into our frame with the proper color space (BGR8)
    try {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<cv::RotatedRect> armors = search(frame, lowerHSV, upperHSV, lowerHSV2, upperHSV2);
    frame_count++;

    if (armors.size() == 2)
    {
        auto p0 = rect_to_point(armors[0]);
        auto p1 = rect_to_point(armors[1]);
        std::cout << frame_count << "," << p0[0] << "," << p0[1] << "," << p1[0] << "," << p1[1] << std::endl;

        draw_rotated_rect(frame, armors[0]);
        draw_rotated_rect(frame, armors[1]);
    }
    else
    {
        std::cout << frame_count << "," << "no armor found" << std::endl;
    }

    // Reduce the computational load and just show every 5th image
    if (frame_count % 5 == 0)
    {
        show_frame(frame);
    }
}

/*
 *  This method displays a frame to your screen.
 */
void ArmorDetectorNode::show_frame(cv::Mat &frame)
{
    std::vector<uchar> buf;
    cv::resize(frame, frame, cv::Size(640, 480));
    cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 20});
    cv::imshow("Detection Frame", cv::imdecode(buf, cv::IMREAD_COLOR));
    if (cv::waitKey(1) == 27)
    {
        cv::destroyAllWindows();
        rclcpp::shutdown();
    }
}

/*
 *  The main method activates this subscriber node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}

/*
 *  This method will search a frame for armor plates and return the RotatedRect objects that correspond to the two light
 *  bars that exist on an armor plate.
 */
std::vector<cv::RotatedRect> ArmorDetectorNode::search(cv::Mat& frame, cv::Scalar lowerHSV, cv::Scalar upperHSV, cv::Scalar lowerHSV2, cv::Scalar upperHSV2) {
    // 1) Image Preprocessing
    // BGR to HSV conversion
    cv::Mat hsv_frame;
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
    // Remove noise with Gaussian blur
    cv::GaussianBlur(hsv_frame, hsv_frame, cv::Size(5, 5), 0);

    // 2) Color segmentation
    cv::Mat maskRange1, maskRange2, mask;
    cv::inRange(hsv_frame, lowerHSV, upperHSV, maskRange1);
    cv::inRange(hsv_frame, lowerHSV2, upperHSV2, maskRange2);
    cv::bitwise_or(maskRange1, maskRange2, mask);

    //  2.5) Edge Detection
    // Necessity depends on the success of color segmentation (try experimenting!)
    // Detect the edges of the isolated regions
    // OpenCV provides cv::Canny()
    // Configure Canny parameters as necessary
    // This removes lingering noise and best captures the important regions
    // Look into morphology in OpenCV if your edge detection isn’t working well enough
    cv::Mat edges;
    cv::Canny(mask, edges, 100, 200);

    // 3) Contour Detection
    // Find arrays of points that represent the outlines of regions
    // OpenCV provides cv::findContours()
    // Configure a minimum contour area to ignore regions that are too small to be important
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::RotatedRect> light_bars;
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) >= 25) { // Minimum contour area threshold
            cv::RotatedRect rect = cv::minAreaRect(contour);
            if (is_light_bar(rect)) {
                light_bars.push_back(rect);
            }
        }
    }

    // 4) Contour Analysis
    // Iterate through your contours to analyze them
    // Fit a rotated rectangle to each contour
    // OpenCV provides cv::RotatedRect, cv::minAreaRect(), and cv::fitEllipse()
    // Find the contour arrays that actually represent the light bars
    // You will write a method that checks if a contour area can be a light bar
    // Find the pair of light bars that represent the armor plate of interest
    // You will write a method that checks if a light bar pair can be an armor plate
    // Draw the armor plate’s light bar contours on the image

    for (size_t i = 0; i < light_bars.size(); i++) {
        for (size_t j = i + 1; j < light_bars.size(); j++) {
            if (is_armor(light_bars[i], light_bars[j])) {
                return {light_bars[i], light_bars[j]};
            }
        }
    }

    return {}; // Default return value, no armor found
}

/*
 *  This method draws a rotated rectangle onto a frame. This is used to display the results of your algorithm when you
 *  run the node.
 */
void ArmorDetectorNode::draw_rotated_rect(cv::Mat &frame, cv::RotatedRect &rect)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }
}

/*
 *  This method determines whether a RotatedRect object can represent a light bar based on the constants defined in the
 *  header file. It checks dimensions, angles, and ratios against our configured thresholds to do so.
 */
bool ArmorDetectorNode::is_light_bar(cv::RotatedRect &rect)
{
    // You may want to read the OpenCV documentation for RotatedRect

    // Verify that the light bar width is valid
    if (rect.size.width < LIGHT_BAR_WIDTH_LOWER_LIMIT) {
        return false;
    }

    // Verify that the light bar height is valid
    if (rect.size.height < LIGHT_BAR_HEIGHT_LOWER_LIMIT) {
        return false;
    }
    // Verify that the light bar angle is valid
    // You will want to compare against both the limit and its supplement; think about the unit circle
    if (rect.angle < -LIGHT_BAR_ANGLE_LIMIT && rect.angle > -(180 - LIGHT_BAR_ANGLE_LIMIT)) {
        return false;
    }
    
    // Verify that the light bar aspect ratio is valid
    // Aspect ratio refers to height / width, not width / height
    float aspect_ratio = rect.size.height / rect.size.width;
    if (aspect_ratio < LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT) {
        return false;
    }
    return true;
}

/*
 *  This method determines whether a pair of light bars (RotatedRect objects) can represent an armor plate based on the
 *  constants defined in the header file. It checks dimensions, angles, and ratios against our configured thresholds to
 *  do so.
 */
bool ArmorDetectorNode::is_armor(cv::RotatedRect &left_rect, cv::RotatedRect &right_rect)
{
    // Verify that the light bars are roughly parallel by checking that their difference does not exceed the threshold
    // Again, you will want to compare against both the limit and its supplement
    if (std::abs(left_rect.angle - right_rect.angle) > ARMOR_ANGLE_DIFF_LIMIT &&
        std::abs(std::abs(left_rect.angle - right_rect.angle) - 180) > ARMOR_ANGLE_DIFF_LIMIT) {
        return false;
    }

    // Verify that the ratio between the light bar aspect ratios (that's a mouthful) is within the threshold
    // You will want to compare both left / right and right / left against the threshold
    float left_aspect_ratio = left_rect.size.height / left_rect.size.width;
    float right_aspect_ratio = right_rect.size.height / right_rect.size.width;
    float aspect_ratio_ratio = left_aspect_ratio / right_aspect_ratio;
    if (aspect_ratio_ratio < (1.0f / ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT) || aspect_ratio_ratio > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT) {
        return false;
    }

    // Verify that the light bars are at roughly the same elevation (as in their y difference is within the threshold)
    // The way the constant was determined assumes that you normalize this difference using the average light bar height
    // What that means is that the expression you should be checking is abs(y_left - y_right) / avg_height
    float avg_height = (left_rect.size.height + right_rect.size.height) / 2.0f;
    if (std::abs(left_rect.center.y - right_rect.center.y) / avg_height > ARMOR_Y_DIFF_LIMIT) {
        return false;
    }

    // Verify that the ratio between light bar heights is within the threshold
    // Again, you will want to compare both left / right and right / left
    float height_ratio = left_rect.size.height / right_rect.size.height;
    if (height_ratio < (1.0f / ARMOR_HEIGHT_RATIO_LIMIT) || height_ratio > ARMOR_HEIGHT_RATIO_LIMIT) {
        return false;
    }

    // Verify that the armor aspect ratio is within the threshold
    // For some goofy reason, the constant for this step requires that you calculate aspect ratio as width / height
    // There are multiple ways to define armor plate "height" and "width." Hopefully your idea is effective!
    float armor_width = std::abs(left_rect.center.x - right_rect.center.x);
    float armor_height = (left_rect.size.height + right_rect.size.height) / 2.0f;
    float armor_aspect_ratio = armor_width / armor_height;
    if (armor_aspect_ratio > ARMOR_ASPECT_RATIO_LIMIT) {
        return false;
    }
    return true;
}

/*
 *  This method represents a RotatedRect object as a point and returns it. It exists for debugging output while the node
 *  is being run.
 */
std::vector<cv::Point2f> ArmorDetectorNode::rect_to_point(cv::RotatedRect &rect)
{
    float rad = rect.angle < 90 ? rect.angle * M_PI / 180.f : (rect.angle - 180) * M_PI / 180.f;
    float x_offset = rect.size.height * std::sin(rad) / 2.f;
    float y_offset = rect.size.height * std::cos(rad) / 2.f;

    std::vector<cv::Point2f> points;
    points = std::vector<cv::Point2f>();
    points.push_back(cv::Point2f(int(rect.center.x + x_offset), int(rect.center.y - y_offset)));
    points.push_back(cv::Point2f(int(rect.center.x - x_offset), int(rect.center.y + y_offset)));
    return points;
}