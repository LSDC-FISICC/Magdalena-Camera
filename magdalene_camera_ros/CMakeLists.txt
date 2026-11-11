cmake_minimum_required(VERSION 3.16)
project(magdalene_camera_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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
    std_msgs
    cv_bridge
    image_transport
    magdalene_camera_sdk
)
target_link_libraries(magdalene_camera_node
    magdalene_camera_sdk::magdalene_camera
)
# Load librealsense from the directory the SDK was built against (/usr/local, with DDS), even when
# the ROS environment puts ros-humble-librealsense2 (same soname, no DDS) first in LD_LIBRARY_PATH.
# DT_RPATH, unlike DT_RUNPATH, is searched before LD_LIBRARY_PATH.
if(realsense2_DIR)
    get_filename_component(MAGDALENE_REALSENSE_LIBDIR "${realsense2_DIR}/../.." ABSOLUTE)
else()
    set(MAGDALENE_REALSENSE_LIBDIR /usr/local/lib)
endif()
message(STATUS "librealsense from ${MAGDALENE_REALSENSE_LIBDIR} (RPATH)")
set_target_properties(magdalene_camera_node PROPERTIES
    BUILD_RPATH "${MAGDALENE_REALSENSE_LIBDIR}" INSTALL_RPATH "${MAGDALENE_REALSENSE_LIBDIR}")
target_link_options(magdalene_camera_node PRIVATE "LINKER:--disable-new-dtags")

install(TARGETS magdalene_camera_node
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
