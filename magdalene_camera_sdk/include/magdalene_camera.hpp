/**
 * @file magdalene_camera.hpp
 * @brief Public interface for the magdalene_camera RealSense driver.
 * @author Julio Fajardo, PhD
 */

#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <stdexcept>

class magdalene_camera {
public:
    struct StreamConfig {
        int width;
        int height;
        int fps;
    };

    // Default: color 640x360 @ 15fps only; right infrared stream and IR projector OFF
    // serial: RealSense device serial number (empty = first available device)
    // infra2_enabled: also stream the right infrared image (infra2_cfg is ignored when false)
    explicit magdalene_camera(
        StreamConfig color_cfg         = {640, 360, 15},
        StreamConfig infra2_cfg        = {640, 360, 15},
        bool         projector_enabled = false,
        std::string  serial            = "",
        bool         infra2_enabled    = false
    );

    ~magdalene_camera();

    // --- Config (call before start()) ---
    void set_color_config(int width, int height, int fps);
    void set_infra2_config(int width, int height, int fps);

    // --- Runtime control (call after start()) ---
    void set_projector_enabled(bool enabled);

    void start();
    void stop();
    bool is_running() const;

    // --- Frame acquisition ---
    // Returns true and fills the Mats if frames arrive within timeout_ms;
    // infra2_out is left empty when the infrared stream is disabled
    bool wait_for_frames(cv::Mat& color_out, cv::Mat& infra2_out,
                         int timeout_ms = 5000);

    // --- Getters ---
    StreamConfig color_config()      const { return color_cfg_; }
    StreamConfig infra2_config()     const { return infra2_cfg_; }
    bool         projector_enabled() const { return projector_enabled_; }
    bool         infra2_enabled()    const { return infra2_enabled_; }
    std::string  serial_number()     const { return serial_; }

private:
    // Seconds to keep retrying the pipeline start while an Ethernet camera is being discovered
    static constexpr int kStartTimeoutS = 15;

    StreamConfig color_cfg_;
    StreamConfig infra2_cfg_;
    bool         projector_enabled_;
    bool         infra2_enabled_;
    bool         running_;
    std::string  serial_;

    // Context with Ethernet (DDS) discovery enabled: librealsense 2.58 leaves it off by default,
    // so PoE cameras would not be found. USB cameras are still enumerated as before.
    rs2::context          ctx_;
    rs2::pipeline         pipeline_;
    rs2::config           rs_cfg_;
    rs2::pipeline_profile profile_;

    rs2::sensor find_stereo_sensor() const;
    void        apply_projector_setting();
    void        configure_streams();
};
