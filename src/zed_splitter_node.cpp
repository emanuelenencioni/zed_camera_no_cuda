#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <regex>

// Structure to hold camera calibration parameters
struct CamParams {
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    double k1 = 0.0, k2 = 0.0, k3 = 0.0, p1 = 0.0, p2 = 0.0;
};

class ZedCameraNode : public rclcpp::Node
{
public:
    ZedCameraNode() : Node("zed_splitter_node")
    {
        // === 1. Declare and get parameters ===
        this->declare_parameter<std::string>("conf_file_path", "");
        this->declare_parameter<std::string>("video_device", "/dev/video2");
        this->declare_parameter<int>("frame_width", 3840); // e.g., 3840 for 2K/FHD, 2560 for HD
        this->declare_parameter<int>("frame_height", 1080); // e.g., 1080 for FHD, 720 for HD
        this->declare_parameter<int>("fps", 30);

        std::string conf_path = this->get_parameter("conf_file_path").as_string();
        std::string video_device = this->get_parameter("video_device").as_string();
        int frame_width = this->get_parameter("frame_width").as_int();
        int frame_height = this->get_parameter("frame_height").as_int();
        int fps = this->get_parameter("fps").as_int();

        if (conf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Configuration file path ('conf_file_path') is not set!");
            rclcpp::shutdown();
            return;
        }

        // === 2. Load calibration data ===
        load_conf_file(conf_path);

        // === 3. Setup OpenCV VideoCapture ===
        cap_.open(video_device, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video device: %s", video_device.c_str());
            rclcpp::shutdown();
            return;
        }

        // Set camera properties
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
        cap_.set(cv::CAP_PROP_FPS, fps);
        // The ZED camera often outputs in MJPEG or YUYV format.
        // MJPEG is compressed and requires more CPU to decode but less USB bandwidth.
        // YUYV is raw and requires more USB bandwidth but less CPU.
        // You might need to experiment with this setting.
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        RCLCPP_INFO(this->get_logger(), "Opened %s with %dx%d @ %d FPS",
            video_device.c_str(), frame_width, frame_height, fps);
        
        // === 4. Create publishers ===
        left_pub_ = image_transport::create_publisher(this, "/left/image_raw");
        right_pub_ = image_transport::create_publisher(this, "/right/image_raw");
        left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/left/camera_info", 10);
        right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/right/camera_info", 10);

        // === 5. Create a timer to drive the capture loop ===
        auto frame_duration = std::chrono::duration<double>(1.0 / fps);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(frame_duration),
            std::bind(&ZedCameraNode::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        // Grab a frame
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Dropped frame");
            return;
        }

        // Split the image
        int width = frame.cols;
        int height = frame.rows;
        int half_width = width / 2;

        if (width == 0 || height == 0) {
             RCLCPP_WARN(this->get_logger(), "Received empty frame");
            return;
        }

        cv::Mat left_image(frame, cv::Rect(0, 0, half_width, height));
        cv::Mat right_image(frame, cv::Rect(half_width, 0, half_width, height));

        // Get common header and resolution key
        auto header = std_msgs::msg::Header();
        header.stamp = this->get_clock()->now();
        std::string res_key = get_resolution_key(height);

        if (res_key.empty()) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported resolution: %dx%d", half_width, height);
            return;
        }

        // Get camera info messages
        auto left_info_msg = get_camera_info(header, "left", res_key, left_image.size());
        auto right_info_msg = get_camera_info(header, "right", res_key, right_image.size());

        // Convert OpenCV Mats back to ROS image messages
        // NOTE: The encoding from V4L2 might be BGR8, but if you set it to YUYV,
        // you might get a different encoding here. We assume BGR8 which is common after OpenCV decodes MJPEG.
        cv_bridge::CvImage left_cv_image(header, "bgr8", left_image);
        cv_bridge::CvImage right_cv_image(header, "bgr8", right_image);

        // Publish everything
        left_pub_.publish(*left_cv_image.toImageMsg());
        right_pub_.publish(*right_cv_image.toImageMsg());
        left_info_pub_->publish(left_info_msg);
        right_info_pub_->publish(right_info_msg);
    }

    // (The following helper functions are the same as before)
    void load_conf_file(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open configuration file: %s", file_path.c_str());
            return;
        }
        
        std::string line;
        std::string current_section;
        std::regex section_regex("\\[(LEFT_CAM|RIGHT_CAM|STEREO)_(.*)\\]");
        std::regex param_regex("(.+?)=(.+)");
        std::smatch match;

        while (getline(file, line)) {
            if (std::regex_match(line, match, section_regex)) {
                current_section = match[0].str();
            } else if (std::regex_match(line, match, param_regex)) {
                if (!current_section.empty()) {
                    params_map_[current_section][match[1].str()] = std::stod(match[2].str());
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Successfully parsed calibration file: %s", file_path.c_str());
    }

    std::string get_resolution_key(int height) {
        if (height == 1080) return "FHD";
        if (height == 720) return "HD";
        if (height == 480) return "VGA";
        if (height == 1242) return "2K";
        return "";
    }

    sensor_msgs::msg::CameraInfo get_camera_info(const std_msgs::msg::Header& header, const std::string& side, const std::string& res_key, const cv::Size& size)
    {
        sensor_msgs::msg::CameraInfo info_msg;
        std::string section_name = (side == "left" ? "[LEFT_CAM_" : "[RIGHT_CAM_") + res_key + "]";
        
        auto& params = params_map_[section_name];

        info_msg.header = header;
        info_msg.header.frame_id = side + "_camera_frame";
        info_msg.height = size.height;
        info_msg.width = size.width;
        
        info_msg.distortion_model = "plumb_bob";
        info_msg.d = {params["k1"], params["k2"], params["p1"], params["p2"], params["k3"]};

        info_msg.k = {params["fx"], 0, params["cx"], 0, params["fy"], params["cy"], 0, 0, 1};

        info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

        double baseline = params_map_["[STEREO_]"]["Baseline"] / 1000.0; // Convert mm to meters
        double tx = (side == "left") ? 0.0 : -params["fx"] * baseline;
        
        info_msg.p = {params["fx"], 0, params["cx"], tx, 0, params["fy"], params["cy"], 0, 0, 0, 1, 0};

        return info_msg;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    image_transport::Publisher left_pub_, right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_, right_info_pub_;
    std::map<std::string, std::map<std::string, double>> params_map_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedCameraNode>());
    rclcpp::shutdown();
    return 0;
}