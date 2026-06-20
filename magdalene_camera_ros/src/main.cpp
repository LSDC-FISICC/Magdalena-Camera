/**
 * @file main.cpp
 * @brief Entry point for the ROS 2 magdalene_camera_node.
 * @author Julio Fajardo, PhD
 *
 * Initializes the ROS 2 context, instantiates MagdaleneCameraNode (which internally
 * opens the RealSense camera and launches the capture thread), and keeps the node
 * alive via rclcpp::spin(). Any unhandled exception is logged as FATAL and triggers
 * a clean shutdown of the ROS 2 context.
 */

#include "magdalene_camera_ros/magdalene_camera_node.hpp"

/**
 * @brief Starts the camera node and blocks until a shutdown signal is received.
 * @param argc Number of command-line arguments (passed to ROS 2).
 * @param argv Array of command-line argument strings.
 * @return 0 on normal shutdown, 1 if a fatal exception is caught.
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);          // Initialize the global ROS 2 context
    try {
        // Create the node and keep it alive processing callbacks
        rclcpp::spin(std::make_shared<MagdaleneCameraNode>());
    } catch (const std::exception & e) {
        // Log the error before shutting down to aid diagnosis
        RCLCPP_FATAL(rclcpp::get_logger("magdalene_camera_node"),
                     "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
