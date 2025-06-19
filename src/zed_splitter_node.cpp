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

class ZedSplitterNode : public rclcpp::Node
{
public:
    ZedSplitterNode() : Node("zed_splitter_node")
    {
        // Declare parameter for the configuration file path
        this->declare_parameter<std::string>("conf_file_path", "");
        std::string conf_path = this->get_parameter("conf_file_path").as_string();

        if (conf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Configuration file path ('conf_file_path') is not set!");
            rclcpp::shutdown();
            return;
        }

        // Load calibration data
        load_conf_file(conf_path);

        // Create publishers
        left_pub_ = image_transport::create_publisher(this, "/left/image_raw");
        right_pub_ = image_transport::create_publisher(this, "/right/image_raw");
        left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/left/camera_info", 10);
        right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/right/camera_info", 10);

        // Create subscriber
        // The QoS profile is set to match typical camera driver outputs
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        sub_ = image_transport::create_subscription(this, "/image_raw", 
            std::bind(&ZedSplitterNode::image_callback, this, std::placeholders::_1), "raw", qos.get_rmw_qos_profile());
        
        RCLCPP_INFO(this->get_logger(), "ZED Splitter node started. Subscribed to '/image_raw'.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // Convert ROS image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Split the image
        int width = cv_ptr->image.cols;
        int height = cv_ptr->image.rows;
        int half_width = width / 2;

        cv::Mat left_image(cv_ptr->image, cv::Rect(0, 0, half_width, height));
        cv::Mat right_image(cv_ptr->image, cv::Rect(half_width, 0, half_width, height));

        // Get the resolution key (e.g., "HD", "FHD") from the image height
        std::string res_key = get_resolution_key(height);
        if (res_key.empty()) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported resolution: %dx%d", half_width, height);
            return;
        }

        // Get camera info messages
        auto left_info_msg = get_camera_info(msg->header, "left", res_key, left_image.size());
        auto right_info_msg = get_camera_info(msg->header, "right", res_key, right_image.size());

        // Convert OpenCV Mats back to ROS image messages
        cv_bridge::CvImage left_cv_image(msg->header, "bgr8", left_image);
        cv_bridge::CvImage right_cv_image(msg->header, "bgr8", right_image);

        // Publish everything
        left_pub_.publish(left_cv_image.toImageMsg());
        right_pub_.publish(right_cv_image.toImageMsg());
        left_info_pub_->publish(left_info_msg);
        right_info_pub_->publish(right_info_msg);
    }

    // Parses the .conf file
    void load_conf_file(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open configuration file: %s", file_path.c_str());
            return;
        }
        
        std::string line;
        std::string current_section;
        std::regex section_regex("\\[(LEFT_CAM|RIGHT_CAM|STEREO)_(.+?)\\]");
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
        if (height == 1242) return "2K"; // ZED2i 2K mode height
        return "";
    }

    // Creates and populates a CameraInfo message
    sensor_msgs::msg::CameraInfo get_camera_info(const std_msgs::msg::Header& header, const std::string& side, const std::string& res_key, const cv::Size& size)
    {
        sensor_msgs::msg::CameraInfo info_msg;
        std::string section_name = (side == "left" ? "[LEFT_CAM_" : "[RIGHT_CAM_") + res_key + "]";
        
        auto& params = params_map_[section_name];

        info_msg.header = header;
        info_msg.header.frame_id = side + "_camera_frame";
        info_msg.height = size.height;
        info_msg.width = size.width;
        
        // Distortion parameters [k1, k2, p1, p2, k3]
        info_msg.distortion_model = "plumb_bob";
        info_msg.d.resize(5);
        info_msg.d[0] = params["k1"];
        info_msg.d[1] = params["k2"];
        info_msg.d[2] = params["p1"];
        info_msg.d[3] = params["p2"];
        info_msg.d[4] = params["k3"];

        // Intrinsic camera matrix K
        info_msg.k[0] = params["fx"]; info_msg.k[1] = 0;          info_msg.k[2] = params["cx"];
        info_msg.k[3] = 0;          info_msg.k[4] = params["fy"]; info_msg.k[5] = params["cy"];
        info_msg.k[6] = 0;          info_msg.k[7] = 0;          info_msg.k[8] = 1;

        // Rectification matrix R (identity for raw images)
        info_msg.r[0] = 1.0; info_msg.r[1] = 0.0; info_msg.r[2] = 0.0;
        info_msg.r[3] = 0.0; info_msg.r[4] = 1.0; info_msg.r[5] = 0.0;
        info_msg.r[6] = 0.0; info_msg.r[7] = 0.0; info_msg.r[8] = 1.0;

        // Projection matrix P
        double baseline = params_map_["[STEREO_]"]["Baseline"] / 1000.0; // Convert mm to meters
        double tx = (side == "left") ? 0.0 : -params["fx"] * baseline;
        
        info_msg.p[0] = params["fx"]; info_msg.p[1] = 0;          info_msg.p[2] = params["cx"]; info_msg.p[3] = tx;
        info_msg.p[4] = 0;          info_msg.p[5] = params["fy"]; info_msg.p[6] = params["cy"]; info_msg.p[7] = 0;
        info_msg.p[8] = 0;          info_msg.p[9] = 0;          info_msg.p[10] = 1;         info_msg.p[11] = 0;

        return info_msg;
    }

    image_transport::Publisher left_pub_, right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_, right_info_pub_;
    image_transport::Subscriber sub_;
    std::map<std::string, std::map<std::string, double>> params_map_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedSplitterNode>());
    rclcpp::shutdown();
    return 0;
}