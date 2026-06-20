#pragma once

// Forward declaration: keeps librealsense2 headers out of the ROS2 main TU
class magdalene_camera;

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <thread>

class MagdaleneCameraNode : public rclcpp::Node
{
public:
    explicit MagdaleneCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MagdaleneCameraNode();

private:
    void capture_loop();

    std::unique_ptr<magdalene_camera>         cam_;
    std::optional<image_transport::Publisher> color_pub_;
    std::optional<image_transport::Publisher> infra2_pub_;
    std::thread                               capture_thread_;
    std::atomic<bool>                         running_{false};
    std::string                               frame_id_;
};
