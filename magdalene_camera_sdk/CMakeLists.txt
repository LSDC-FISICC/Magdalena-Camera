cmake_minimum_required(VERSION 3.16)
project(magdalene_camera_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED HINTS /usr/local/lib/cmake/realsense2)

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
