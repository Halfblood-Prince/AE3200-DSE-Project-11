cmake_minimum_required(VERSION 3.21)

project(ros_test
    VERSION 0.1.0
    DESCRIPTION "Gazebo Jetty 3D lidar robot simulation with ROS 2 Lyrical Luth OctoMap mapping"
    LANGUAGES NONE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/website/cmake")

set(ROS_TEST_TARGET_ROS_DISTRO "lyrical")
set(ROS_TEST_TARGET_UBUNTU_CODENAME "resolute")

find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
    message(FATAL_ERROR
        "ament_cmake was not found. Install ROS 2 Lyrical dependencies on "
        "Ubuntu 26.04 and source /opt/ros/lyrical/setup.bash before running "
        "colcon build.")
endif()

set(ROS_TEST_PYTHON_EXECUTABLES
    auto_drive
    map_filter
    map_monitor
    nav2_waypoint_explorer
    odom_to_tf)

foreach(executable IN LISTS ROS_TEST_PYTHON_EXECUTABLES)
    install(PROGRAMS "scripts/${executable}.py"
        DESTINATION lib/${PROJECT_NAME}
        RENAME "${executable}")
endforeach()

set(ROS_TEST_PREBUILT_EXECUTABLES web_server)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ROS_TEST_SYSTEM_PROCESSOR)
set(ROS_TEST_BIN_ARCH "")
if(ROS_TEST_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(ROS_TEST_BIN_ARCH "amd_x64")
elseif(ROS_TEST_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(ROS_TEST_BIN_ARCH "arm_x64")
endif()

set(ROS_TEST_PREBUILT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin/${ROS_TEST_BIN_ARCH}")
set(ROS_TEST_PREBUILT_LIB_DIR "${ROS_TEST_PREBUILT_DIR}/lib")
set(ROS_TEST_PREBUILT_MANIFEST "${ROS_TEST_PREBUILT_DIR}/build-info.env")

option(
    ROS_TEST_USE_PREBUILT_BINARIES
    "Install shipped web_server and dependency libraries from bin/<arch> instead of compiling the Drogon server"
    ON)

function(ros_test_target_dependencies target)
    foreach(dependency IN LISTS ARGN)
        if(TARGET "${dependency}::${dependency}")
            target_link_libraries(${target} PRIVATE "${dependency}::${dependency}")
        elseif(DEFINED ${dependency}_TARGETS)
            target_link_libraries(${target} PRIVATE ${${dependency}_TARGETS})
        elseif(DEFINED ${dependency}_LIBRARIES)
            target_link_libraries(${target} PRIVATE ${${dependency}_LIBRARIES})
            if(DEFINED ${dependency}_INCLUDE_DIRS)
                target_include_directories(${target} PRIVATE ${${dependency}_INCLUDE_DIRS})
            endif()
        else()
            message(FATAL_ERROR "No CMake target or library variable found for dependency ${dependency}")
        endif()
    endforeach()
endfunction()

function(require_prebuilt_metadata key expected)
    file(STRINGS "${ROS_TEST_PREBUILT_MANIFEST}" matches REGEX "^${key}=")
    if(NOT matches)
        message(FATAL_ERROR "Prebuilt bundle metadata ${key} is missing from ${ROS_TEST_PREBUILT_MANIFEST}")
    endif()

    list(GET matches 0 metadata_line)
    string(REGEX REPLACE "^${key}=(.*)$" "\\1" actual "${metadata_line}")
    if(NOT actual STREQUAL expected)
        message(FATAL_ERROR
            "Prebuilt bundle metadata ${key}=${actual}, expected ${expected}. "
            "Regenerate bin/${ROS_TEST_BIN_ARCH} for ROS 2 ${ROS_TEST_TARGET_ROS_DISTRO} "
            "on Ubuntu ${ROS_TEST_TARGET_UBUNTU_CODENAME}.")
    endif()
endfunction()

if(ROS_TEST_USE_PREBUILT_BINARIES)
    if(NOT ROS_TEST_BIN_ARCH)
        message(FATAL_ERROR "No prebuilt binary folder is defined for CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}")
    endif()

    set(ROS_TEST_PREBUILT_PROGRAMS "")
    foreach(executable IN LISTS ROS_TEST_PREBUILT_EXECUTABLES)
        if(NOT EXISTS "${ROS_TEST_PREBUILT_DIR}/${executable}")
            message(FATAL_ERROR "Missing prebuilt executable: ${ROS_TEST_PREBUILT_DIR}/${executable}")
        endif()
        list(APPEND ROS_TEST_PREBUILT_PROGRAMS "${ROS_TEST_PREBUILT_DIR}/${executable}")
    endforeach()

    if(NOT EXISTS "${ROS_TEST_PREBUILT_MANIFEST}")
        message(FATAL_ERROR
            "Missing prebuilt bundle metadata: ${ROS_TEST_PREBUILT_MANIFEST}. "
            "Push to ros_simulation, wait for the AeroSentinel Linux Binaries "
            "workflow to publish bin/${ROS_TEST_BIN_ARCH}/build-info.env, and "
            "pull that commit. Maintainers can configure with "
            "-DROS_TEST_USE_PREBUILT_BINARIES=OFF to build the Drogon server from source.")
    endif()
    require_prebuilt_metadata("ROS_DISTRO" "${ROS_TEST_TARGET_ROS_DISTRO}")
    require_prebuilt_metadata("UBUNTU_CODENAME" "${ROS_TEST_TARGET_UBUNTU_CODENAME}")

    if(NOT IS_DIRECTORY "${ROS_TEST_PREBUILT_LIB_DIR}")
        message(FATAL_ERROR "Missing prebuilt dependency bundle: ${ROS_TEST_PREBUILT_LIB_DIR}")
    endif()

    file(GLOB ROS_TEST_PREBUILT_DEPENDENCIES CONFIGURE_DEPENDS
        "${ROS_TEST_PREBUILT_LIB_DIR}/*.so*")
    if(NOT ROS_TEST_PREBUILT_DEPENDENCIES)
        message(FATAL_ERROR "No prebuilt shared libraries found in ${ROS_TEST_PREBUILT_LIB_DIR}")
    endif()

    message(STATUS "Installing prebuilt ros_test web_server and dependency bundle from ${ROS_TEST_PREBUILT_DIR}")
    install(PROGRAMS
        ${ROS_TEST_PREBUILT_PROGRAMS}
        DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY "${ROS_TEST_PREBUILT_LIB_DIR}/"
        DESTINATION lib/${PROJECT_NAME}/lib
        USE_SOURCE_PERMISSIONS
        FILES_MATCHING PATTERN "*.so*")
else()
    enable_language(CXX)

    find_package(geometry_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    include(AeroSentinelMySQLHints)
    find_package(Drogon CONFIG REQUIRED)
    find_package(FFmpeg REQUIRED COMPONENTS AVCODEC AVUTIL SWSCALE)
    find_package(LibDataChannel CONFIG REQUIRED)
    find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

    add_executable(web_server website/src/main.cc)
    target_link_libraries(web_server
        PRIVATE
            Drogon::Drogon
            FFmpeg::avcodec
            FFmpeg::avutil
            FFmpeg::swscale
            LibDataChannel::LibDataChannel
            ${OpenCV_LIBS})
    target_include_directories(web_server
        PRIVATE
            ${OpenCV_INCLUDE_DIRS})
    ros_test_target_dependencies(web_server
        rclcpp
        geometry_msgs
        sensor_msgs)
    target_compile_definitions(web_server
        PRIVATE
            AEROSENTINEL_VERSION="${PROJECT_VERSION}")
    set_target_properties(web_server
        PROPERTIES
            BUILD_RPATH "$ORIGIN/lib;$ORIGIN"
            INSTALL_RPATH "$ORIGIN/lib;$ORIGIN")
    add_custom_command(TARGET web_server POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/website/public"
            "$<TARGET_FILE_DIR:web_server>/public")
    install(TARGETS web_server
        RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

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
