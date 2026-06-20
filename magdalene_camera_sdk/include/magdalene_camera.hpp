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

    // Default: 640x360 @ 15fps, projector OFF
    // serial: RealSense device serial number (empty = first available device)
    explicit magdalene_camera(
        StreamConfig color_cfg         = {640, 360, 15},
        StreamConfig infra2_cfg        = {640, 360, 15},
        bool         projector_enabled = false,
        std::string  serial            = ""
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
    // Returns true and fills both Mats if frames arrive within timeout_ms
    bool wait_for_frames(cv::Mat& color_out, cv::Mat& infra2_out,
                         int timeout_ms = 5000);

    // --- Getters ---
    StreamConfig color_config()      const { return color_cfg_; }
    StreamConfig infra2_config()     const { return infra2_cfg_; }
    bool         projector_enabled() const { return projector_enabled_; }
    std::string  serial_number()     const { return serial_; }

private:
    StreamConfig color_cfg_;
    StreamConfig infra2_cfg_;
    bool         projector_enabled_;
    bool         running_;
    std::string  serial_;

    rs2::pipeline         pipeline_;
    rs2::config           rs_cfg_;
    rs2::pipeline_profile profile_;

    rs2::sensor find_stereo_sensor() const;
    void        apply_projector_setting();
    void        configure_streams();
};
