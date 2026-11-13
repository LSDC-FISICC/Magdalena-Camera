/**
 * @file magdalene_camera_node.cpp
 * @brief Implementation of MagdaleneCameraNode, a ROS 2 wrapper for the RealSense camera.
 * @author Julio Fajardo, PhD
 *
 * Declares ROS 2 parameters, instantiates the magdalene_camera driver, and
 * publishes color frames (and, when infra2_enabled is set, infrared frames) on
 * separate image_transport topics from a
 * dedicated capture thread.
 *
 * A stream watchdog measures the color rate once per second (published on
 * color/fps). When the rate stays below watchdog_min_fraction x color_fps for
 * watchdog_low_s seconds, or its mean over the last watchdog_window_s seconds
 * falls below watchdog_window_fraction x color_fps (a camera that degrades to
 * 10-13 fps for minutes before it stalls), the node shuts down with exit code kWatchdogExitCode, and the launch file starts
 * it again (respawn) in a fresh process. Over PoE, two cameras whose frame
 * bursts overlap at the switch lose most of their frames until one of them
 * restarts; restarting the process shifts that camera's phase and both recover
 * (see the README). Restarting the pipeline inside the same process did not
 * bring Ethernet cameras back, so the whole process is restarted.
 */

#include "magdalene_camera_ros/magdalene_camera_node.hpp"
#include <magdalene_camera.hpp>
#include <algorithm>
#include <cstdio>

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
    declare_parameter("infra2_enabled", false);   // color only by default
    declare_parameter("color_width",   640);
    declare_parameter("color_height",  360);
    declare_parameter("color_fps",     15);
    declare_parameter("infra2_width",  640);
    declare_parameter("infra2_height", 360);
    declare_parameter("infra2_fps",    15);
    // Stream watchdog: exit (and be respawned) when the color rate stays below fraction x color_fps
    declare_parameter("watchdog_min_fraction", 0.67);  // 0.0 disables this rule (color/fps is still published)
    declare_parameter("watchdog_low_s",        3);     // seconds below the threshold before an exit is scheduled
    // Second rule: mean rate over a sliding window (catches slow degradation that never stays low 3 s)
    declare_parameter("watchdog_window_fraction", 0.9);  // 13.5 fps at 15 fps; 0.0 disables this rule
    declare_parameter("watchdog_window_s",        30);   // window length in seconds
    declare_parameter("watchdog_jitter_s",     5.0);   // random wait before exiting; cancelled if the rate recovers
    declare_parameter("watchdog_grace_s",      20.0);  // no decision this long after the start

    auto serial    = get_parameter("serial").as_string();
    frame_id_      = get_parameter("frame_id").as_string();
    bool projector = get_parameter("projector").as_bool();
    bool infra2_enabled = get_parameter("infra2_enabled").as_bool();

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

    cam_ = std::make_unique<magdalene_camera>(color_cfg, infra2_cfg, projector, serial, infra2_enabled);

    // Advertise image topics using image_transport for compressed-transport support
    color_pub_.emplace(image_transport::create_publisher(this, "color/image_raw"));
    if (infra2_enabled)
        infra2_pub_.emplace(image_transport::create_publisher(this, "infra2/image_raw"));

    fps_pub_ = create_publisher<std_msgs::msg::Float32>("color/fps", 10);
    min_fps_      = std::max(0.0, get_parameter("watchdog_min_fraction").as_double()) * color_cfg.fps;
    low_s_needed_ = std::max<int>(1, get_parameter("watchdog_low_s").as_int());
    jitter_s_     = std::max(0.0, get_parameter("watchdog_jitter_s").as_double());
    window_fps_   = std::max(0.0, get_parameter("watchdog_window_fraction").as_double()) * color_cfg.fps;
    window_s_     = std::max<int>(1, get_parameter("watchdog_window_s").as_int());
    const double grace_s = std::max(0.0, get_parameter("watchdog_grace_s").as_double());
    rng_.seed(std::random_device{}());

    cam_->start();
    running_ = true;
    capture_thread_ = std::thread(&MagdaleneCameraNode::capture_loop, this);
    last_tick_   = Clock::now();
    grace_until_ = last_tick_ + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(grace_s));
    watchdog_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { watchdog_tick(); });
    if (min_fps_ > 0 || window_fps_ > 0)
        RCLCPP_INFO(get_logger(), "Stream watchdog: exit for respawn below %.1f fps for %d s or below a mean of "
                    "%.1f fps over %d s (jitter %.1f s, grace %.0f s)", min_fps_, low_s_needed_, window_fps_,
                    window_s_, jitter_s_, grace_s);

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
            ++frames_;

            auto now = get_clock()->now();
            std_msgs::msg::Header hdr;
            hdr.stamp    = now;
            hdr.frame_id = frame_id_;

            color_pub_->publish(*cv_bridge::CvImage(hdr, "bgr8",  color).toImageMsg());
            if (infra2_pub_ && !infra2.empty())
                infra2_pub_->publish(*cv_bridge::CvImage(hdr, "mono8", infra2).toImageMsg());
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Frame capture error: %s", e.what());
        }
    }
}

/**
 * @brief Once per second: publishes the color rate and decides whether to exit.
 *
 * Two rules schedule an exit: the rate stays below watchdog_min_fraction x
 * color_fps for watchdog_low_s consecutive seconds (a camera that stalls), or
 * its mean over the last watchdog_window_s seconds falls below
 * watchdog_window_fraction x color_fps (a camera that degrades for minutes,
 * oscillating between 10 and 15 fps, before it stalls). The exit comes after a
 * random delay of up to watchdog_jitter_s and is cancelled if the rate is back
 * at the window threshold for two consecutive seconds. With two cameras failing
 * together, the first one to restart usually frees the other, whose pending
 * exit is then cancelled. The exit shuts ROS down; main() returns
 * kWatchdogExitCode and the launch file respawns the node.
 */
void MagdaleneCameraNode::watchdog_tick()
{
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    const double fps = dt > 0 ? frames_.exchange(0) / dt : 0.0;

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(fps);
    fps_pub_->publish(msg);

    if ((min_fps_ <= 0 && window_fps_ <= 0) || exit_requested_ || now < grace_until_)
        return;

    window_.push_back(fps);
    while (static_cast<int>(window_.size()) > window_s_)
        window_.pop_front();
    low_s_ = (min_fps_ > 0 && fps < min_fps_) ? low_s_ + 1 : 0;

    if (pending_) {
        const double ok_fps = window_fps_ > 0 ? window_fps_ : min_fps_;
        ok_s_ = fps >= ok_fps ? ok_s_ + 1 : 0;
        if (ok_s_ >= 2) {
            RCLCPP_INFO(get_logger(), "Color rate back to %.1f fps, restart cancelled", fps);
            pending_ = false;
            low_s_ = 0;
            window_.clear();        // the window refills before it can trigger again
            return;
        }
    } else {
        char why[96] = "";
        if (low_s_ >= low_s_needed_) {
            std::snprintf(why, sizeof(why), "%.1f fps below %.1f for %d s", fps, min_fps_, low_s_);
        } else if (window_fps_ > 0 && static_cast<int>(window_.size()) >= window_s_) {
            double sum = 0;
            for (double v : window_) sum += v;
            const double mean = sum / window_.size();
            if (mean < window_fps_)
                std::snprintf(why, sizeof(why), "mean %.1f fps over %d s below %.1f", mean, window_s_, window_fps_);
        }
        if (why[0]) {
            std::uniform_real_distribution<double> wait(0.0, jitter_s_);
            const double w = wait(rng_);
            exit_at_ = now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(w));
            pending_ = true;
            ok_s_ = 0;
            RCLCPP_WARN(get_logger(), "Color rate %s; restarting the node in %.1f s unless it recovers", why, w);
        }
    }
    if (pending_ && now >= exit_at_) {
        exit_requested_ = true;
        RCLCPP_WARN(get_logger(), "Restarting the node (exit code %d, respawned by the launch file)",
                    kWatchdogExitCode);
        rclcpp::shutdown();
    }
}
