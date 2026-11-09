/**
 * @file magdalene_camera_node.hpp
 * @brief ROS 2 node interface for the magdalene_camera RealSense driver.
 * @author Julio Fajardo, PhD
 */

#pragma once

// Forward declaration: keeps librealsense2 headers out of the ROS2 main TU
class magdalene_camera;

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>

class MagdaleneCameraNode : public rclcpp::Node
{
public:
    explicit MagdaleneCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MagdaleneCameraNode();

private:
    using Clock = std::chrono::steady_clock;

    void capture_loop();
    void watchdog_tick();     // 1 Hz wall timer: measures the color rate, decides restarts
    void restart_stream();    // runs on the capture thread: stop + start the pipeline

    std::unique_ptr<magdalene_camera>         cam_;
    std::optional<image_transport::Publisher> color_pub_;
    std::optional<image_transport::Publisher> infra2_pub_;
    std::thread                               capture_thread_;
    std::atomic<bool>                         running_{false};
    std::string                               frame_id_;

    // --- Stream watchdog ---
    // The capture thread counts frames; the timer turns the count into a rate, publishes
    // it on color/fps and, when the rate stays low, asks the capture thread to restart.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fps_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::atomic<uint32_t>     frames_{0};
    std::atomic<bool>         restart_requested_{false};
    std::atomic<bool>         restarting_{false};
    std::atomic<int64_t>      grace_until_ns_{0};   // steady-clock ns; no restart decisions before it
    double                    min_fps_{0.0};        // 0 disables restarts
    int                       low_s_needed_{3};
    double                    jitter_s_{3.0};
    double                    grace_s_{15.0};
    int                       max_per_min_{3};
    // Timer-thread state
    int                       low_s_{0};
    bool                      pending_{false};
    Clock::time_point         restart_at_{};
    Clock::time_point         last_tick_{};
    std::deque<Clock::time_point> restarts_;
    std::mt19937              rng_;
    int                       restart_count_{0};
};
