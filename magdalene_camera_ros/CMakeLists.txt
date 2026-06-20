cmake_minimum_required(VERSION 3.16)
project(magdalene_camera_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(magdalene_camera_sdk REQUIRED)

add_executable(magdalene_camera_node
    src/main.cpp
    src/magdalene_camera_node.cpp
)
target_include_directories(magdalene_camera_node PRIVATE include)
ament_target_dependencies(magdalene_camera_node
    rclcpp
    sensor_msgs
    cv_bridge
    image_transport
    magdalene_camera_sdk
)
target_link_libraries(magdalene_camera_node
    magdalene_camera_sdk::magdalene_camera
)

install(TARGETS magdalene_camera_node
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
