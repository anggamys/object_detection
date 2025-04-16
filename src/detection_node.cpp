#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>
#include <fstream>

class DetectionNode : public rclcpp::Node
{
public:
    DetectionNode() : Node("detection_node")
    {
        // Declare parameter untuk path model
        this->declare_parameter<std::string>("model_path", "/home/c0delb08/ros2_ws/src/object_detection/models/yolov8n.onnx");
        
        // Ambil nilai parameter
        std::string model_path = this->get_parameter("model_path").as_string();
        
        // Coba load model dengan ONNX Runtime
        try {
            // Initialize ONNX Runtime environment
            env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "detection_node");
            
            // Set session options
            Ort::SessionOptions session_options;
            session_options.SetIntraOpNumThreads(1);
            session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
            
            // Create session
            session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options);
            
            // Check if model loaded successfully
            if (!session_) {
                RCLCPP_FATAL(this->get_logger(), "Model kosong atau gagal diload: %s", model_path.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Model berhasil diload dengan ONNX Runtime: %s", model_path.c_str());
                
                // Get model info
                Ort::AllocatorWithDefaultOptions allocator;
                size_t num_input_nodes = session_->GetInputCount();
                size_t num_output_nodes = session_->GetOutputCount();
                
                RCLCPP_INFO(this->get_logger(), "Input nodes: %ld, Output nodes: %ld", num_input_nodes, num_output_nodes);
                
                // Print input node info
                for (size_t i = 0; i < num_input_nodes; i++) {
                    auto input_name = session_->GetInputNameAllocated(i, allocator);
                    RCLCPP_INFO(this->get_logger(), "Input %ld: %s", i, input_name.get());
                }
                
                // Print output node info
                for (size_t i = 0; i < num_output_nodes; i++) {
                    auto output_name = session_->GetOutputNameAllocated(i, allocator);
                    RCLCPP_INFO(this->get_logger(), "Output %ld: %s", i, output_name.get());
                }
            }
        } catch (const Ort::Exception& e) {
            RCLCPP_FATAL(this->get_logger(), "ONNX Runtime error saat load model: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Error saat load model: %s", e.what());
        }
    }

private:
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}