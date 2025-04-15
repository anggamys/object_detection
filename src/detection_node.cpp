#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class DetectionNode : public rclcpp::Node 
{
public:
    DetectionNode()
        : Node("detection_node"), model_loaded_(false)
    {
        // Declare parameters (Default values)
        this->declare_parameter<bool>("use_gpu", false);
        this->declare_parameter<std::string>("model_path", "/home/c0delb08/ros2_ws/src/object_detection/models/yolov8n.onnx");
        this->declare_parameter<double>("confidence_threshold", 0.4);

        bool use_gpu = this->get_parameter("use_gpu").as_bool();
        std::string model_path = this->get_parameter("model_path").as_string();
        confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();

        try {
            net_ = cv::dnn::readNetFromONNX(model_path);
            if (net_.empty()) {
                throw std::runtime_error("Model file could not be loaded or is empty.");
            }

            if (use_gpu) {
                net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
                RCLCPP_INFO(this->get_logger(), "Using GPU (CUDA) for inference");
            } else {
                net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                RCLCPP_INFO(this->get_logger(), "Using CPU for inference");
            }

            model_loaded_ = true;
            RCLCPP_INFO(this->get_logger(), "YOLO model loaded: %s", model_path.c_str());
        } catch (const cv::Exception& e) {
            RCLCPP_FATAL(this->get_logger(), "OpenCV error loading model: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Error loading model: %s", e.what());
        }

        if (model_loaded_) {
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/image_raw", 10,
                std::bind(&DetectionNode::image_callback, this, std::placeholders::_1));

            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_detected", 10);
        } else {
            RCLCPP_FATAL(this->get_logger(), "Model not loaded. Node will not subscribe or publish.");
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::dnn::Net net_;
    bool model_loaded_;
    double confidence_threshold_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!model_loaded_) return;

        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty frame");
            return;
        }

        // Preprocess image
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);
        net_.setInput(blob);

        // Run forward pass
        std::vector<cv::Mat> outputs;
        try {
            net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Model inference failed: %s", e.what());
            return;
        }

        const auto& detection = outputs[0];
        const int rows = detection.size[1];
        const int cols = detection.size[2];
        const float* data = reinterpret_cast<float*>(detection.data);

        for (int i = 0; i < rows; ++i) {
            float objectness = data[4];
            float* class_scores = const_cast<float*>(data + 5);
            cv::Mat scores(1, cols - 5, CV_32FC1, class_scores);

            cv::Point class_id_point;
            double max_class_score;
            cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);

            float confidence = objectness * static_cast<float>(max_class_score);
            if (confidence < confidence_threshold_) {
                data += cols;
                continue;
            }

            // Get bounding box
            int cx = static_cast<int>(data[0] * frame.cols);
            int cy = static_cast<int>(data[1] * frame.rows);
            int w  = static_cast<int>(data[2] * frame.cols);
            int h  = static_cast<int>(data[3] * frame.rows);

            int left = std::max(cx - w / 2, 0);
            int top  = std::max(cy - h / 2, 0);
            int right = std::min(left + w, frame.cols - 1);
            int bottom = std::min(top + h, frame.rows - 1);

            cv::rectangle(frame, cv::Rect(left, top, right - left, bottom - top), cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, 
                        "ID: " + std::to_string(class_id_point.x), 
                        cv::Point(left, top - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                        cv::Scalar(0, 255, 0), 1);

            data += cols;
        }

        // Publish the image
        auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        publisher_->publish(*output_msg);
    }
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
