#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CompressedCameraPublisherNode : public rclcpp::Node {
public:
    CompressedCameraPublisherNode() : Node("compressed_camera_publisher_node"), is_running_(true) {
        // Declare Parameters
        this->declare_parameter("width", 1536);
        this->declare_parameter("height", 864);
        this->declare_parameter("fps", 30);
        this->declare_parameter("jpeg_quality", 80);
        this->declare_parameter("frame_id", "camera_optical_frame");
        this->declare_parameter("topic_name", "/camera/image_raw/compressed");
        this->declare_parameter("device", "/dev/video0");

        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        std::string topic_name = this->get_parameter("topic_name").as_string();
        device_ = this->get_parameter("device").as_string();

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name, 5);

        // Try opening with Pi 5 libcamera pipeline first
        std::string pipeline = "libcamerasrc ! video/x-raw, width=" + std::to_string(width_) +
                               ", height=" + std::to_string(height_) +
                               ", framerate=" + std::to_string(fps_) + "/1" + 
                               ", format=RGBx ! videoconvert ! video/x-raw, format=BGR ! appsink max-buffers=1 drop=true sync=false";
                               
        cap_.open(pipeline, cv::CAP_GSTREAMER);
        
        // Fallback to standard USB/V4L2 if libcamera fails
        if (!cap_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "libcamera failed. Falling back to V4L2 USB camera.");
            int device_id = 0; // Default to 0, extract from string if needed
            if (device_.length() > 10 && device_.substr(0, 10) == "/dev/video") {
                device_id = std::stoi(device_.substr(10));
            }
            cap_.open(device_id, cv::CAP_V4L2);
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
            cap_.set(cv::CAP_PROP_FPS, fps_);
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open any camera! Check connections.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Camera opened successfully. Starting C++ capture thread.");

        // Start capture thread to prevent blocking the ROS 2 executor
        capture_thread_ = std::thread(&CompressedCameraPublisherNode::capture_worker, this);
    }

    ~CompressedCameraPublisherNode() {
        is_running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    void capture_worker() {
        cv::Mat frame;
        std::vector<uchar> buffer;
        std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

        // Pre-allocate buffer to save memory reallocation overhead
        buffer.reserve(width_ * height_ * 3); 

        while (rclcpp::ok() && is_running_) {
            cap_ >> frame;
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Dropped frame. Retrying...");
                std::this_thread::sleep_for(50ms);
                continue;
            }

            // Encode to JPEG in memory
            cv::imencode(".jpg", frame, buffer, encode_params);

            // Build and publish message
            auto msg = sensor_msgs::msg::CompressedImage();
            msg.header.stamp = this->now();
            msg.header.frame_id = frame_id_;
            msg.format = "jpeg";
            msg.data = buffer;

            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    cv::VideoCapture cap_;
    std::thread capture_thread_;
    std::atomic<bool> is_running_;
    
    int width_;
    int height_;
    int fps_;
    int jpeg_quality_;
    std::string frame_id_;
    std::string device_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompressedCameraPublisherNode>());
    rclcpp::shutdown();
    return 0;
}