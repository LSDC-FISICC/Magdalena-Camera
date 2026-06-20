/**
 * @file magdalene_camera_wrapper.cpp
 * @brief Implementation of the magdalene_camera RealSense driver.
 * @author Julio Fajardo, PhD
 *
 * Wraps the librealsense2 pipeline to stream color (BGR8) and right-infrared
 * (Y8) frames. Provides runtime control over stream resolution/FPS and the
 * IR dot projector emitter.
 */

#include "magdalene_camera.hpp"
#include <iostream>

/**
 * @brief Constructs the camera object with the given stream configurations.
 * @param color_cfg   Resolution and FPS for the color stream.
 * @param infra2_cfg  Resolution and FPS for the right infrared stream.
 * @param projector_enabled Whether the IR dot projector should be on at start.
 * @param serial      Device serial number; empty string selects any device.
 */
magdalene_camera::magdalene_camera(StreamConfig color_cfg,
                                   StreamConfig infra2_cfg,
                                   bool         projector_enabled,
                                   std::string  serial)
    : color_cfg_(color_cfg)
    , infra2_cfg_(infra2_cfg)
    , projector_enabled_(projector_enabled)
    , running_(false)
    , serial_(std::move(serial))
{}

/**
 * @brief Destructor — stops the pipeline if it is still running.
 */
magdalene_camera::~magdalene_camera()
{
    if (running_) stop();
}

/**
 * @brief Updates the color stream resolution and FPS.
 * @param width  Frame width in pixels.
 * @param height Frame height in pixels.
 * @param fps    Target frame rate.
 * @throws std::runtime_error if called while the camera is running.
 */
void magdalene_camera::set_color_config(int width, int height, int fps)
{
    if (running_)
        throw std::runtime_error("Cannot change config while camera is running");
    color_cfg_ = {width, height, fps};
}

/**
 * @brief Updates the right infrared stream resolution and FPS.
 * @param width  Frame width in pixels.
 * @param height Frame height in pixels.
 * @param fps    Target frame rate.
 * @throws std::runtime_error if called while the camera is running.
 */
void magdalene_camera::set_infra2_config(int width, int height, int fps)
{
    if (running_)
        throw std::runtime_error("Cannot change config while camera is running");
    infra2_cfg_ = {width, height, fps};
}

/**
 * @brief Populates the librealsense config with the color and infrared streams.
 *
 * If a serial number was provided at construction the config is bound to that
 * specific device. Otherwise any connected RealSense is accepted.
 */
void magdalene_camera::configure_streams()
{
    rs_cfg_ = rs2::config();

    if (!serial_.empty())
        rs_cfg_.enable_device(serial_);

    rs_cfg_.enable_stream(RS2_STREAM_COLOR,
                          0,
                          color_cfg_.width, color_cfg_.height,
                          RS2_FORMAT_BGR8,
                          color_cfg_.fps);

    // Infrared index 2 = right stereo camera
    rs_cfg_.enable_stream(RS2_STREAM_INFRARED,
                          2,
                          infra2_cfg_.width, infra2_cfg_.height,
                          RS2_FORMAT_Y8,
                          infra2_cfg_.fps);
}

/**
 * @brief Starts the librealsense pipeline and applies the projector setting.
 * @throws std::runtime_error if the camera is already running or librealsense
 *         reports an error during pipeline start.
 */
void magdalene_camera::start()
{
    if (running_)
        throw std::runtime_error("Camera is already running");

    configure_streams();

    try {
        profile_ = pipeline_.start(rs_cfg_);
    } catch (const rs2::error& e) {
        throw std::runtime_error(std::string("librealsense error on start: ") + e.what());
    }

    running_ = true;
    apply_projector_setting();

    std::cout << "[magdalene_camera] started — "
              << "color " << color_cfg_.width << "x" << color_cfg_.height
              << "@" << color_cfg_.fps << "fps | "
              << "infra2 " << infra2_cfg_.width << "x" << infra2_cfg_.height
              << "@" << infra2_cfg_.fps << "fps | "
              << "projector " << (projector_enabled_ ? "ON" : "OFF") << "\n";
}

/**
 * @brief Stops the librealsense pipeline and marks the camera as idle.
 */
void magdalene_camera::stop()
{
    if (!running_) return;
    pipeline_.stop();
    running_ = false;
    std::cout << "[magdalene_camera] stopped\n";
}

/** @brief Returns true if the pipeline is currently streaming. */
bool magdalene_camera::is_running() const { return running_; }

/**
 * @brief Waits for the next synchronized frameset and decodes it into cv::Mat.
 *
 * Blocks for up to @p timeout_ms milliseconds. Returns false (without throwing)
 * if the timeout expires or either frame is invalid, so the caller can simply
 * retry on the next iteration.
 *
 * @param color_out   Output BGR8 color frame.
 * @param infra2_out  Output Y8 right-infrared frame.
 * @param timeout_ms  Maximum wait time in milliseconds.
 * @returns true if both frames were decoded successfully, false on timeout or
 *          invalid frame.
 * @throws std::runtime_error if called while the camera is not running.
 */
bool magdalene_camera::wait_for_frames(cv::Mat& color_out, cv::Mat& infra2_out,
                                       int timeout_ms)
{
    if (!running_)
        throw std::runtime_error("Camera is not running");

    rs2::frameset frameset;
    if (!pipeline_.try_wait_for_frames(&frameset, static_cast<unsigned int>(timeout_ms)))
        return false;

    rs2::video_frame color_frame = frameset.get_color_frame();
    if (!color_frame || color_frame.get_width() <= 0 || color_frame.get_height() <= 0)
        return false;
    // Clone so the cv::Mat owns its data independently of the RealSense frame buffer
    color_out = cv::Mat(
        color_frame.get_height(), color_frame.get_width(),
        CV_8UC3, const_cast<void*>(color_frame.get_data())
    ).clone();

    rs2::video_frame infra2_frame = frameset.get_infrared_frame(2);
    if (!infra2_frame || infra2_frame.get_width() <= 0 || infra2_frame.get_height() <= 0)
        return false;
    infra2_out = cv::Mat(
        infra2_frame.get_height(), infra2_frame.get_width(),
        CV_8UC1, const_cast<void*>(infra2_frame.get_data())
    ).clone();

    return true;
}

/**
 * @brief Finds the depth/stereo sensor on the active device.
 * @returns The first sensor that exposes rs2::depth_sensor capabilities.
 * @throws std::runtime_error if no depth sensor is found.
 */
rs2::sensor magdalene_camera::find_stereo_sensor() const
{
    for (auto& sensor : profile_.get_device().query_sensors()) {
        if (sensor.is<rs2::depth_sensor>())
            return sensor;
    }
    throw std::runtime_error("Stereo/depth sensor not found on device");
}

/**
 * @brief Enables or disables the IR dot projector on the stereo sensor.
 *
 * Failures are logged as warnings rather than thrown, since some devices or
 * firmware versions do not expose the emitter control option.
 */
void magdalene_camera::apply_projector_setting()
{
    try {
        auto sensor = find_stereo_sensor();
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED,
                              projector_enabled_ ? 1.0f : 0.0f);
    } catch (const rs2::error& e) {
        std::cerr << "[magdalene_camera] warning — projector control failed: "
                  << e.what() << "\n";
    }
}

/**
 * @brief Toggles the IR dot projector and applies the change immediately if running.
 * @param enabled true to enable the projector, false to disable it.
 */
void magdalene_camera::set_projector_enabled(bool enabled)
{
    projector_enabled_ = enabled;
    if (running_) apply_projector_setting();
    std::cout << "[magdalene_camera] IR projector "
              << (enabled ? "enabled" : "disabled") << "\n";
}
