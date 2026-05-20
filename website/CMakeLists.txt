cmake_minimum_required(VERSION 3.21)

project(aerosentinel_control_center
    VERSION 1.0.0
    DESCRIPTION "AeroSentinel Control Center built with Drogon C++ and ROS 2"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Drogon CONFIG REQUIRED)
find_package(FFmpeg REQUIRED COMPONENTS AVCODEC AVUTIL SWSCALE)
find_package(LibDataChannel CONFIG REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

function(aerosentinel_target_dependencies target)
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

add_executable(aerosentinel-control
    src/main.cc)

target_link_libraries(aerosentinel-control
    PRIVATE
        Drogon::Drogon
        FFmpeg::avcodec
        FFmpeg::avutil
        FFmpeg::swscale
        LibDataChannel::LibDataChannel
        ${OpenCV_LIBS})

target_include_directories(aerosentinel-control
    PRIVATE
        ${OpenCV_INCLUDE_DIRS})

aerosentinel_target_dependencies(aerosentinel-control
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
