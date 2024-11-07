#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "/home/prachit/Projects/colcon_ws/install/ultralytics_ros/include/ultralytics_ros/ultralytics_ros/msg/yolo_result.hpp"
#include <string>

class YoloListener : public rclcpp::Node {
public:
    YoloListener() : Node("yolo_listener_cpp") {
        // Create a subscription to the /yolo_result topic
        subscription_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
            "/yolo_result", 10, std::bind(&YoloListener::yoloCallback, this, std::placeholders::_1));
    }

private:
    void yoloCallback(const ultralytics_ros::msg::YoloResult::SharedPtr msg) {
        for (const auto& detection : msg->detections.detections) {
            for (const auto& result : detection.results) {
                // Extract class ID and score
                std::string class_id = result.hypothesis.class_id;
                float score = result.hypothesis.score;

                // Extract bounding box information
                float bbox_center_x = detection.bbox.center.position.x;
                float bbox_center_y = detection.bbox.center.position.y;
                float bbox_size_x = detection.bbox.size_x;
                float bbox_size_y = detection.bbox.size_y;

                // Log detection details
                RCLCPP_INFO(this->get_logger(), "Detected object: %s with confidence %.2f", class_id.c_str(), score);
                RCLCPP_INFO(this->get_logger(), "Bounding box center: (x=%.2f, y=%.2f)", bbox_center_x, bbox_center_y);
                RCLCPP_INFO(this->get_logger(), "Bounding box size: (width=%.2f, height=%.2f)", bbox_size_x, bbox_size_y);
            }
        }
    }

    rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloListener>());
    rclcpp::shutdown();
    return 0;
}
