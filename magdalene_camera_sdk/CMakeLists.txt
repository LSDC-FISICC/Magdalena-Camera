cmake_minimum_required(VERSION 3.16)
project(magdalene_camera_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED HINTS /usr/local/lib/cmake/realsense2)

# librealsense with Ethernet (DDS) support is built into /usr/local. The ROS 2 package
# ros-humble-librealsense2 (no DDS) ships the same soname librealsense2.so.2.58 since 2.58.4, and
# the ROS environment puts it first in LD_LIBRARY_PATH, so PoE cameras would not be found. The
# binaries therefore carry an RPATH (DT_RPATH, which unlike DT_RUNPATH wins over LD_LIBRARY_PATH)
# to the directory of the librealsense found here.
get_filename_component(MAGDALENE_REALSENSE_LIBDIR "${realsense2_DIR}/../.." ABSOLUTE)
message(STATUS "librealsense from ${MAGDALENE_REALSENSE_LIBDIR} (RPATH)")

add_library(magdalene_camera src/magdalene_camera_wrapper.cpp)
target_include_directories(magdalene_camera PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(magdalene_camera PUBLIC
    ${realsense2_LIBRARY}
    ${OpenCV_LIBS}
)

add_executable(magdalene_camera_demo src/main.cpp)
target_link_libraries(magdalene_camera_demo magdalene_camera)
foreach(t magdalene_camera magdalene_camera_demo)
    set_target_properties(${t} PROPERTIES
        BUILD_RPATH "${MAGDALENE_REALSENSE_LIBDIR}" INSTALL_RPATH "${MAGDALENE_REALSENSE_LIBDIR}")
    target_link_options(${t} PRIVATE "LINKER:--disable-new-dtags")
endforeach()

install(TARGETS magdalene_camera
    EXPORT export_magdalene_camera
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS magdalene_camera_demo
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_magdalene_camera HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV realsense2)

ament_package()
