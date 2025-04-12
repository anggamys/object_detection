#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <stdexcept>

using std::placeholders::_1;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
    : Node("camera_node")
    {
        // Parameter with default value
        this->declare_parameter<std::string>("source", "0");

        // Get parameter from command line
        std::string source = this->get_parameter("source").as_string();

        // Check if source is a number (camera) or string (video file)
        if (std::all_of(source.begin(), source.end(), ::isdigit)) {
            int camera_index = std::stoi(source);
            cap_.open(camera_index);
        } else {
            // Try opening with default backend first
            cap_.open(source);
            if (!cap_.isOpened()) {
                // If failed, try with FFMPEG
                cap_.open(source, cv::CAP_FFMPEG);
            }
        }

        if (!cap_.isOpened()) {
            std::string error_msg = "Failed to open video source: " + source;
            RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
            throw std::runtime_error(error_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened video source: %s", source.c_str());
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(kTopicName, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), 
                                       std::bind(&CameraNode::publish_frame, this));
    }

private:
    static constexpr const char* kTopicName = "/image_raw";
    static constexpr const char* kFrameId = "camera_frame";

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_frame()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Empty frame");
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = kFrameId;

        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }
};

int main(int argc, char **argv)
{
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<CameraNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}