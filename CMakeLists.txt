cmake_minimum_required(VERSION 3.21)

project(ros_test
    VERSION 0.1.0
    DESCRIPTION "Gazebo Harmonic lidar robot simulation with ROS 2 Jazzy SLAM Toolbox mapping"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/website/cmake")

find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
    message(FATAL_ERROR
        "ament_cmake was not found. Install ROS 2 Jazzy dependencies and source "
        "/opt/ros/jazzy/setup.bash before running colcon build.")
endif()

set(ROS_TEST_PREBUILT_EXECUTABLES
    auto_drive
    map_filter
    map_monitor
    nav2_waypoint_explorer
    odom_to_tf
    scan_to_chassis
    simple_mapper)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ROS_TEST_SYSTEM_PROCESSOR)
set(ROS_TEST_BIN_ARCH "")
if(ROS_TEST_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(ROS_TEST_BIN_ARCH "amd_x64")
elseif(ROS_TEST_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(ROS_TEST_BIN_ARCH "arm_x64")
endif()

set(ROS_TEST_PREBUILT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin/${ROS_TEST_BIN_ARCH}")
set(ROS_TEST_PREBUILT_COMPLETE OFF)
if(ROS_TEST_BIN_ARCH)
    set(ROS_TEST_PREBUILT_COMPLETE ON)
    foreach(executable IN LISTS ROS_TEST_PREBUILT_EXECUTABLES)
        if(NOT EXISTS "${ROS_TEST_PREBUILT_DIR}/${executable}")
            set(ROS_TEST_PREBUILT_COMPLETE OFF)
        endif()
    endforeach()
endif()

option(
    ROS_TEST_USE_PREBUILT_BINARIES
    "Install prebuilt non-web executables from bin/<arch> instead of compiling those package executables"
    ${ROS_TEST_PREBUILT_COMPLETE})

function(add_ros_executable target source)
    add_executable(${target} ${source})
    ament_target_dependencies(${target} ${ARGN})
    install(TARGETS ${target}
        RUNTIME DESTINATION lib/${PROJECT_NAME})
endfunction()

if(ROS_TEST_USE_PREBUILT_BINARIES)
    if(NOT ROS_TEST_BIN_ARCH)
        message(FATAL_ERROR "No prebuilt binary folder is defined for CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}")
    endif()

    foreach(executable IN LISTS ROS_TEST_PREBUILT_EXECUTABLES)
        if(NOT EXISTS "${ROS_TEST_PREBUILT_DIR}/${executable}")
            message(FATAL_ERROR "Missing prebuilt executable: ${ROS_TEST_PREBUILT_DIR}/${executable}")
        endif()
    endforeach()

    message(STATUS "Installing prebuilt non-web ros_test executables from ${ROS_TEST_PREBUILT_DIR}")
    install(PROGRAMS
        ${ROS_TEST_PREBUILT_DIR}/auto_drive
        ${ROS_TEST_PREBUILT_DIR}/map_filter
        ${ROS_TEST_PREBUILT_DIR}/map_monitor
        ${ROS_TEST_PREBUILT_DIR}/nav2_waypoint_explorer
        ${ROS_TEST_PREBUILT_DIR}/odom_to_tf
        ${ROS_TEST_PREBUILT_DIR}/scan_to_chassis
        ${ROS_TEST_PREBUILT_DIR}/simple_mapper
        DESTINATION lib/${PROJECT_NAME})
else()
    find_package(geometry_msgs REQUIRED)
    find_package(nav2_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_action REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)

    add_ros_executable(auto_drive src/auto_drive.cpp rclcpp geometry_msgs sensor_msgs)
    add_ros_executable(map_filter src/map_filter.cpp rclcpp nav_msgs)
    add_ros_executable(map_monitor src/map_monitor.cpp rclcpp nav_msgs)
    add_ros_executable(nav2_waypoint_explorer
        src/nav2_waypoint_explorer.cpp
        rclcpp
        rclcpp_action
        geometry_msgs
        nav2_msgs
        nav_msgs
        tf2
        tf2_ros)
    add_ros_executable(odom_to_tf src/odom_to_tf.cpp rclcpp geometry_msgs nav_msgs tf2_ros)
    add_ros_executable(scan_to_chassis src/scan_to_chassis.cpp rclcpp sensor_msgs)
    add_ros_executable(simple_mapper src/simple_mapper.cpp rclcpp nav_msgs sensor_msgs)
endif()

find_package(Drogon CONFIG REQUIRED)
find_package(FFmpeg REQUIRED COMPONENTS AVCODEC AVUTIL SWSCALE)
find_package(LibDataChannel CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(web_server website/src/main.cc)
target_link_libraries(web_server
    Drogon::Drogon
    FFmpeg::avcodec
    FFmpeg::avutil
    FFmpeg::swscale
    LibDataChannel::LibDataChannel
    ${OpenCV_LIBS})
target_include_directories(web_server
    PRIVATE
        ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(web_server
    rclcpp
    geometry_msgs
    sensor_msgs)
target_compile_definitions(web_server
    PRIVATE
        AEROSENTINEL_VERSION="${PROJECT_VERSION}")
add_custom_command(TARGET web_server POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/website/public"
        "$<TARGET_FILE_DIR:web_server>/public")
install(TARGETS web_server
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(FILES
    README.md
    robot.sdf
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
    config
    launch
    rviz
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY website/public
    DESTINATION share/${PROJECT_NAME}/website)

install(DIRECTORY website/public
    DESTINATION lib/${PROJECT_NAME})

ament_package()
