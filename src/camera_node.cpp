#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraNode : public rclcpp::Node 
{
public:
    CameraNode() : Node("camera_node"), cap_(0) {
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Kamera gagal dibuka");
            rclcpp::shutdown();
            return;
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&CameraNode::camera_callback, this));
    }

private:
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    void camera_callback()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame kosong");
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
        // RCLCPP_DEBUG(this->get_logger(), "Frame published");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
