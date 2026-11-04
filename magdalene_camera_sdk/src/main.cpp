/**
 * @file main.cpp
 * @brief Standalone viewer for the magdalene_camera SDK.
 * @author Julio Fajardo, PhD
 *
 * Opens a RealSense camera via the magdalene_camera driver and displays the
 * color stream in an OpenCV window; with --infra2 it also shows the right
 * infrared stream and lets the user toggle the IR dot projector at runtime.
 * Intended as a quick sanity-check tool rather than a production binary.
 *
 * Usage: ./magdalene_camera_demo [serial] [--infra2]
 *   serial    Optional device serial number. Omit to use any connected device.
 *   --infra2  Also stream and show the right infrared image (off by default).
 */

#include "magdalene_camera.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

/**
 * @brief Entry point for the standalone camera viewer.
 * @param argc Argument count; an optional serial number may be passed as argv[1].
 * @param argv Argument vector.
 * @return 0 on normal exit, 1 if the camera fails to start.
 */
int main(int argc, char* argv[])
{
    std::string serial;
    bool show_infra2 = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--infra2")
            show_infra2 = true;
        else
            serial = argv[i];
    }

    magdalene_camera cam(
        {640, 360, 15},  // color stream config
        {640, 360, 15},  // right infrared stream config
        false,           // IR projector off by default
        serial,
        show_infra2      // right infrared stream off unless --infra2
    );

    try {
        cam.start();
    } catch (const std::exception& e) {
        std::cerr << "Error starting camera: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Press 'q' to quit" << (show_infra2 ? ", 'p' to toggle IR projector" : "") << "\n";

    cv::Mat color, infra2;

    while (true) {
        // Skip iteration on timeout; the camera remains running
        if (!cam.wait_for_frames(color, infra2)) {
            std::cerr << "Timeout waiting for frames\n";
            continue;
        }

        cv::imshow("Color", color);
        if (!infra2.empty())
            cv::imshow("Infra2", infra2);

        // waitKey(1) yields to the OS for at least 1 ms, required to refresh windows
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27)   // 'q' or Escape to quit
            break;
        if (key == 'p' && show_infra2)
            cam.set_projector_enabled(!cam.projector_enabled());
    }

    cam.stop();
    cv::destroyAllWindows();
    return 0;
}
