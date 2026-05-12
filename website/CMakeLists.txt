cmake_minimum_required(VERSION 3.21)

project(aerosentinel_control_center
    VERSION 1.0.0
    DESCRIPTION "AeroSentinel Control Center built with Drogon C++ and ROS 2"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Drogon CONFIG REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(aerosentinel-control
    src/main.cc)

target_link_libraries(aerosentinel-control
    PRIVATE
        Drogon::Drogon
        ${OpenCV_LIBS})

target_include_directories(aerosentinel-control
    PRIVATE
        ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(aerosentinel-control
    rclcpp
    geometry_msgs
    sensor_msgs)

target_compile_definitions(aerosentinel-control
    PRIVATE
        AEROSENTINEL_VERSION="${PROJECT_VERSION}")

if(WIN32)
    target_compile_definitions(aerosentinel-control
        PRIVATE
            NOMINMAX
            WIN32_LEAN_AND_MEAN)
endif()

add_custom_command(TARGET aerosentinel-control POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/public"
        "$<TARGET_FILE_DIR:aerosentinel-control>/public")

install(TARGETS aerosentinel-control
    RUNTIME DESTINATION .)

install(DIRECTORY public DESTINATION .)
