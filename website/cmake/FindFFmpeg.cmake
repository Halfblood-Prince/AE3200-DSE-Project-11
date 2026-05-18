include(FindPackageHandleStandardArgs)

find_package(PkgConfig QUIET)

function(_ffmpeg_find_component component pkg header library target)
    if(PkgConfig_FOUND)
        pkg_check_modules(PC_FFMPEG_${component} QUIET ${pkg})
    endif()

    find_path(FFMPEG_${component}_INCLUDE_DIR
        NAMES ${header}
        HINTS ${PC_FFMPEG_${component}_INCLUDE_DIRS})

    find_library(FFMPEG_${component}_LIBRARY
        NAMES ${library}
        HINTS ${PC_FFMPEG_${component}_LIBRARY_DIRS})

    if(FFMPEG_${component}_INCLUDE_DIR AND FFMPEG_${component}_LIBRARY)
        set(FFmpeg_${component}_FOUND TRUE PARENT_SCOPE)
        set(FFMPEG_${component}_FOUND TRUE PARENT_SCOPE)

        if(NOT TARGET FFmpeg::${target})
            add_library(FFmpeg::${target} UNKNOWN IMPORTED)
            set_target_properties(FFmpeg::${target} PROPERTIES
                IMPORTED_LOCATION "${FFMPEG_${component}_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${FFMPEG_${component}_INCLUDE_DIR}")
        endif()

        set(FFMPEG_LIBRARIES
            ${FFMPEG_LIBRARIES}
            "${FFMPEG_${component}_LIBRARY}"
            PARENT_SCOPE)
        set(FFMPEG_INCLUDE_DIRS
            ${FFMPEG_INCLUDE_DIRS}
            "${FFMPEG_${component}_INCLUDE_DIR}"
            PARENT_SCOPE)
    else()
        set(FFmpeg_${component}_FOUND FALSE PARENT_SCOPE)
        set(FFMPEG_${component}_FOUND FALSE PARENT_SCOPE)
    endif()
endfunction()

_ffmpeg_find_component(AVCODEC libavcodec libavcodec/avcodec.h avcodec avcodec)
_ffmpeg_find_component(AVUTIL libavutil libavutil/avutil.h avutil avutil)
_ffmpeg_find_component(SWSCALE libswscale libswscale/swscale.h swscale swscale)

list(REMOVE_DUPLICATES FFMPEG_LIBRARIES)
list(REMOVE_DUPLICATES FFMPEG_INCLUDE_DIRS)

find_package_handle_standard_args(FFmpeg
    REQUIRED_VARS FFMPEG_LIBRARIES FFMPEG_INCLUDE_DIRS
    HANDLE_COMPONENTS)
