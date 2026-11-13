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

/// Exit code of a node that the stream watchdog stopped so that the launch file respawns it.
constexpr int kWatchdogExitCode = 3;

class MagdaleneCameraNode : public rclcpp::Node
{
public:
    explicit MagdaleneCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MagdaleneCameraNode();

    /// True when the stream watchdog shut the node down (main() then returns kWatchdogExitCode).
    bool watchdog_exit() const { return exit_requested_; }

private:
    using Clock = std::chrono::steady_clock;

    void capture_loop();
    void watchdog_tick();     // 1 Hz wall timer: measures the color rate, decides the exit

    std::unique_ptr<magdalene_camera>         cam_;
    std::optional<image_transport::Publisher> color_pub_;
    std::optional<image_transport::Publisher> infra2_pub_;
    std::thread                               capture_thread_;
    std::atomic<bool>                         running_{false};
    std::string                               frame_id_;

    // --- Stream watchdog ---
    // The capture thread counts frames; the timer turns the count into a rate, publishes it on
    // color/fps and, when the rate stays low, shuts the node down so that the launch file
    // respawns it in a fresh process. All other members are used by the timer only.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fps_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::atomic<uint32_t>     frames_{0};
    double                    min_fps_{0.0};        // 0 disables the exit
    int                       low_s_needed_{3};
    double                    jitter_s_{5.0};
    double                    window_fps_{0.0};     // mean-rate threshold; 0 disables the window rule
    int                       window_s_{30};
    std::deque<double>        window_;             // per-second rates of the last window_s_ seconds
    int                       low_s_{0};
    int                       ok_s_{0};
    bool                      pending_{false};
    bool                      exit_requested_{false};
    Clock::time_point         grace_until_{};
    Clock::time_point         exit_at_{};
    Clock::time_point         last_tick_{};
    std::mt19937              rng_;
};
