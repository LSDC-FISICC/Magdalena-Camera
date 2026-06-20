/**
 * @file magdalene_camera_node.cpp
 * @brief Implementation of MagdaleneCameraNode, a ROS 2 wrapper for the RealSense camera.
 * @author Julio Fajardo, PhD
 *
 * Declares ROS 2 parameters, instantiates the magdalene_camera driver, and
 * publishes color and infrared frames on separate image_transport topics from a
 * dedicated capture thread.
 */

#include "magdalene_camera_ros/magdalene_camera_node.hpp"
#include <magdalene_camera.hpp>

/**
 * @brief Constructs the node, configures the camera, and starts the capture thread.
 * @param options ROS 2 node options forwarded to the base Node constructor.
 */
MagdaleneCameraNode::MagdaleneCameraNode(const rclcpp::NodeOptions & options)
: Node("magdalene_camera", options)
{
    declare_parameter("serial",        std::string(""));
    declare_parameter("frame_id",      get_name());
    declare_parameter("projector",     false);
    declare_parameter("color_width",   640);
    declare_parameter("color_height",  360);
    declare_parameter("color_fps",     15);
    declare_parameter("infra2_width",  640);
    declare_parameter("infra2_height", 360);
    declare_parameter("infra2_fps",    15);

    auto serial    = get_parameter("serial").as_string();
    frame_id_      = get_parameter("frame_id").as_string();
    bool projector = get_parameter("projector").as_bool();

    // Build stream configurations from declared parameters
    magdalene_camera::StreamConfig color_cfg {
        static_cast<int>(get_parameter("color_width").as_int()),
        static_cast<int>(get_parameter("color_height").as_int()),
        static_cast<int>(get_parameter("color_fps").as_int())
    };
    magdalene_camera::StreamConfig infra2_cfg {
        static_cast<int>(get_parameter("infra2_width").as_int()),
        static_cast<int>(get_parameter("infra2_height").as_int()),
        static_cast<int>(get_parameter("infra2_fps").as_int())
    };

    cam_ = std::make_unique<magdalene_camera>(color_cfg, infra2_cfg, projector, serial);

    // Advertise image topics using image_transport for compressed-transport support
    color_pub_.emplace(image_transport::create_publisher(this, "color/image_raw"));
    infra2_pub_.emplace(image_transport::create_publisher(this, "infra2/image_raw"));

    cam_->start();
    running_ = true;
    capture_thread_ = std::thread(&MagdaleneCameraNode::capture_loop, this);

    RCLCPP_INFO(get_logger(), "Started — serial: %s  frame_id: %s",
                serial.empty() ? "(any)" : serial.c_str(), frame_id_.c_str());
}

/**
 * @brief Stops the capture thread and shuts down the camera driver.
 */
MagdaleneCameraNode::~MagdaleneCameraNode()
{
    running_ = false;
    if (capture_thread_.joinable())
        capture_thread_.join();
    cam_->stop();
}

/**
 * @brief Continuously retrieves frames from the camera and publishes them.
 *
 * Runs on a dedicated thread. Waits up to 500 ms per frame; if no frame arrives
 * within that window the loop retries. Exceptions are caught and logged so that
 * a single bad frame does not crash the node.
 */
void MagdaleneCameraNode::capture_loop()
{
    cv::Mat color, infra2;
    while (running_) {
        try {
            if (!cam_->wait_for_frames(color, infra2, 500))
                continue;

            auto now = get_clock()->now();
            std_msgs::msg::Header hdr;
            hdr.stamp    = now;
            hdr.frame_id = frame_id_;

            color_pub_->publish(*cv_bridge::CvImage(hdr, "bgr8",  color).toImageMsg());
            infra2_pub_->publish(*cv_bridge::CvImage(hdr, "mono8", infra2).toImageMsg());
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Frame capture error: %s", e.what());
        }
    }
}
