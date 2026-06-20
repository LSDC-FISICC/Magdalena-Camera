/**
 * @file main.cpp
 * @brief Standalone viewer for the magdalene_camera SDK.
 *
 * Opens a RealSense camera via the magdalene_camera driver, displays the color
 * and right-infrared streams in separate OpenCV windows, and allows the user to
 * toggle the IR dot projector at runtime. Intended as a quick sanity-check tool
 * rather than a production binary.
 *
 * Usage: ./magdalene_camera_viewer [serial]
 *   serial  Optional device serial number. Omit to use any connected device.
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
    std::string serial = (argc > 1) ? argv[1] : "";

    magdalene_camera cam(
        {640, 360, 15},  // color stream config
        {640, 360, 15},  // right infrared stream config
        false,           // IR projector off by default
        serial
    );

    try {
        cam.start();
    } catch (const std::exception& e) {
        std::cerr << "Error starting camera: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Press 'q' to quit, 'p' to toggle IR projector\n";

    cv::Mat color, infra2;

    while (true) {
        // Skip iteration on timeout; the camera remains running
        if (!cam.wait_for_frames(color, infra2)) {
            std::cerr << "Timeout waiting for frames\n";
            continue;
        }

        cv::imshow("Color", color);
        cv::imshow("Infra2", infra2);

        // waitKey(1) yields to the OS for at least 1 ms, required to refresh windows
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27)   // 'q' or Escape to quit
            break;
        if (key == 'p')
            cam.set_projector_enabled(!cam.projector_enabled());
    }

    cam.stop();
    cv::destroyAllWindows();
    return 0;
}
