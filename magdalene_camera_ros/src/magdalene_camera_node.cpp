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
 * color/fps) and restarts the librealsense pipeline when the rate stays below
 * watchdog_min_fraction x color_fps. Over PoE, two cameras whose frame bursts
 * overlap at the switch lose most of their frames until one of them restarts;
 * a restart shifts that camera's phase and both recover (see the README).
 */

#include "magdalene_camera_ros/magdalene_camera_node.hpp"
#include <magdalene_camera.hpp>
#include <algorithm>

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
    // Stream watchdog: restart when the color rate stays below fraction x color_fps
    declare_parameter("watchdog_min_fraction", 0.67);  // 0.0 disables restarts (color/fps is still published)
    declare_parameter("watchdog_low_s",        3);     // seconds below the threshold before a restart is scheduled
    declare_parameter("watchdog_jitter_s",     3.0);   // random wait before restarting; cancelled if the rate recovers
    declare_parameter("watchdog_grace_s",      15.0);  // no decisions this long after a start or restart
    declare_parameter("watchdog_max_per_min",  3);     // restart limit per camera and minute

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
    grace_s_      = std::max(0.0, get_parameter("watchdog_grace_s").as_double());
    max_per_min_  = std::max<int>(1, get_parameter("watchdog_max_per_min").as_int());
    rng_.seed(std::random_device{}());

    cam_->start();
    grace_until_ns_ = (Clock::now() + std::chrono::duration_cast<Clock::duration>(
                          std::chrono::duration<double>(grace_s_))).time_since_epoch().count();
    running_ = true;
    capture_thread_ = std::thread(&MagdaleneCameraNode::capture_loop, this);
    last_tick_ = Clock::now();
    watchdog_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { watchdog_tick(); });
    if (min_fps_ > 0)
        RCLCPP_INFO(get_logger(), "Stream watchdog: restart below %.1f fps for %d s (jitter %.1f s, "
                    "grace %.0f s, max %d/min)", min_fps_, low_s_needed_, jitter_s_, grace_s_, max_per_min_);

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
        if (restart_requested_ || !cam_->is_running()) {
            restart_stream();
            continue;
        }
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
 * @brief Stops and restarts the librealsense pipeline (capture thread only).
 *
 * Only the capture thread touches the pipeline, so a restart never races with
 * wait_for_frames(). If the start fails (camera unreachable for kStartTimeoutS),
 * the camera stays stopped and the capture loop tries again after a pause.
 */
void MagdaleneCameraNode::restart_stream()
{
    restarting_ = true;
    const auto t0 = Clock::now();
    const bool was_running = cam_->is_running();
    if (was_running) {
        ++restart_count_;
        RCLCPP_WARN(get_logger(), "Restarting the stream (restart %d)", restart_count_);
        try {
            cam_->stop();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Stream stop failed: %s", e.what());
        }
    }
    try {
        cam_->start();
        RCLCPP_INFO(get_logger(), "Stream running again after %.1f s",
                    std::chrono::duration<double>(Clock::now() - t0).count());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Stream start failed, retrying in 2 s: %s", e.what());
        for (int i = 0; i < 20 && running_; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    frames_ = 0;
    grace_until_ns_ = (Clock::now() + std::chrono::duration_cast<Clock::duration>(
                          std::chrono::duration<double>(grace_s_))).time_since_epoch().count();
    restart_requested_ = false;
    restarting_ = false;
}

/**
 * @brief Once per second: publishes the color rate and decides whether to restart.
 *
 * A restart is scheduled after watchdog_low_s consecutive seconds below the
 * threshold, at a random delay of up to watchdog_jitter_s, and cancelled if the
 * rate recovers before then. With two cameras failing together, the first one to
 * restart usually frees the other, whose pending restart is then cancelled.
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

    if (min_fps_ <= 0)
        return;
    if (restarting_ || restart_requested_ || now.time_since_epoch().count() < grace_until_ns_) {
        low_s_ = 0;
        pending_ = false;
        return;
    }
    if (fps >= min_fps_) {
        if (pending_)
            RCLCPP_INFO(get_logger(), "Color rate back to %.1f fps, restart cancelled", fps);
        low_s_ = 0;
        pending_ = false;
        return;
    }
    if (!pending_ && ++low_s_ >= low_s_needed_) {
        std::uniform_real_distribution<double> wait(0.0, jitter_s_);
        const double w = wait(rng_);
        restart_at_ = now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(w));
        pending_ = true;
        RCLCPP_WARN(get_logger(), "Color rate %.1f fps below %.1f for %d s; restart in %.1f s unless it recovers",
                    fps, min_fps_, low_s_, w);
    }
    if (!pending_ || now < restart_at_)
        return;

    while (!restarts_.empty() && now - restarts_.front() > std::chrono::minutes(1))
        restarts_.pop_front();
    pending_ = false;
    low_s_ = 0;
    if (static_cast<int>(restarts_.size()) >= max_per_min_) {
        // Hold off quietly until the oldest restart leaves the one-minute window
        const auto resume = restarts_.front() + std::chrono::minutes(1);
        grace_until_ns_ = resume.time_since_epoch().count();
        RCLCPP_ERROR(get_logger(), "%d restarts in the last minute: next restart allowed in %.0f s",
                     max_per_min_, std::chrono::duration<double>(resume - now).count());
        return;
    }
    restarts_.push_back(now);
    restart_requested_ = true;
}
