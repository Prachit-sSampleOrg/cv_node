#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include </home/nvidia/colcon_ws/install/ultralytics_ros/include/ultralytics_ros/ultralytics_ros/msg/yolo_result.hpp>
#include <string>

class YoloListener : public rclcpp::Node
{
public:
    YoloListener() : Node("yolo_listener_cpp")
    {
        rclcpp::QoS depth_qos(10);
        depth_qos.keep_last(10);
        depth_qos.best_effort();
        depth_qos.durability_volatile();

        subscription_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
            "/yolo_result", 10, std::bind(&YoloListener::yoloCallback, this, std::placeholders::_1)
        );

        // mDepthSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "/zed/zed_node/depth/depth_registered", depth_qos,
        //     std::bind(&YoloListener::depthCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Bool>("stopSignDetected", 10);
    }

private:
    void yoloCallback(const ultralytics_ros::msg::YoloResult msg)
    {
        RCLCPP_INFO(this->get_logger(), "Detected object: with confidence");
        for (const auto &detection : msg.detections.detections)
        {
            for (const auto &result : detection.results)
            {
                // Extract class ID and score
                std::string class_id = result.hypothesis.class_id;
                float score = result.hypothesis.score;

                //Publish isStopSignDetected
                if (class_id == "stop sign" && score >= CONFIDENCE_THRESHOLD ) {
                    publishStopSignDetected();
                }
                
                // Log detection details
                RCLCPP_INFO(this->get_logger(), "Detected object: %s with confidence %.2f", class_id.c_str(), score);
            }
        }
    }

    // void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    // {
    //     float *depths = (float *)(&msg->data[0]);

    //     // float u = bbox_center_x / 2;
    //     // float v = bbox_center_y / 2;
    //     RCLCPP_INFO(get_logger(), "Camera Width %d", msg->width);
    //     int centerIdx = static_cast<int>(bbox_center_x * msg->width + bbox_center_y);
    //     centerIdx *= 4;
    //     RCLCPP_INFO(get_logger(), "%d", centerIdx);
    //     depth = depths[centerIdx];
    //     RCLCPP_INFO(get_logger(), "Center distance : %f m", depth);
    // }

    void publishStopSignDetected()
    {
        auto message = std_msgs::msg::Bool();
        message.data = true;
        publisher_->publish(message);
    }

    rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr subscription_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mDepthSub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    
    float CONFIDENCE_THRESHOLD = .7;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloListener>());
    rclcpp::shutdown();
    return 0;
}