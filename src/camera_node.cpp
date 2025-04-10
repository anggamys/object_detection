#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class CameraNode : public rclcpp::Node 
{
public:
    CameraNode()
        : Node("camera_node"), cap_(0)
    {
        if (!init_camera()) {
            rclcpp::shutdown();
            return;
        }

        publisher_ = this->create_publisher<ImageMsg>(kTopicName, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 FPS
            [this]() { this->publish_frame(); }
        );
    }

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using Publisher = rclcpp::Publisher<ImageMsg>::SharedPtr;
    using Timer = rclcpp::TimerBase::SharedPtr;

    static constexpr const char* kTopicName = "/image_raw";
    static constexpr const char* kFrameId = "camera_frame";

    cv::VideoCapture cap_;
    Publisher publisher_;
    Timer timer_;

    bool init_camera()
    {
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Kamera gagal dibuka");
            return false;
        }
        return true;
    }

    void publish_frame()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Frame kosong");
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = kFrameId;

        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
        RCLCPP_DEBUG(this->get_logger(), "Frame published");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
