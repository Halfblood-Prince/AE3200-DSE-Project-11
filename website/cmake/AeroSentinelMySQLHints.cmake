# Help Drogon's FindMySQL.cmake on Debian/Ubuntu multiarch installs where
# mysql_config may omit an explicit -L path for libmysqlclient.
if(UNIX AND NOT APPLE)
    if(NOT MYSQL_INCLUDE_DIR AND EXISTS "/usr/include/mysql/mysql.h")
        set(MYSQL_INCLUDE_DIR
            "/usr/include/mysql"
            CACHE PATH "MySQL include directory" FORCE)
    endif()

    if(NOT MYSQL_LIB_DIR)
        set(_AEROSENTINEL_MYSQL_SEARCH_PATHS)
        if(CMAKE_LIBRARY_ARCHITECTURE)
            list(APPEND _AEROSENTINEL_MYSQL_SEARCH_PATHS
                "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
        endif()
        list(APPEND _AEROSENTINEL_MYSQL_SEARCH_PATHS
            /usr/lib
            /usr/lib/mysql)

        find_library(_AEROSENTINEL_MYSQL_CLIENT_LIBRARY
            NAMES mysqlclient mariadb
            PATHS ${_AEROSENTINEL_MYSQL_SEARCH_PATHS}
            NO_DEFAULT_PATH)

        if(_AEROSENTINEL_MYSQL_CLIENT_LIBRARY)
            get_filename_component(
                _AEROSENTINEL_MYSQL_CLIENT_LIBRARY_DIR
                "${_AEROSENTINEL_MYSQL_CLIENT_LIBRARY}"
                DIRECTORY)
            set(MYSQL_LIB_DIR
                "${_AEROSENTINEL_MYSQL_CLIENT_LIBRARY_DIR}"
                CACHE PATH "MySQL client library directory" FORCE)
            set(MYSQL_LIBRARY_DIR
                "${_AEROSENTINEL_MYSQL_CLIENT_LIBRARY_DIR}"
                CACHE PATH "MySQL client library directory" FORCE)
            set(MYSQL_LIB
                "${_AEROSENTINEL_MYSQL_CLIENT_LIBRARY}"
                CACHE FILEPATH "MySQL client library" FORCE)
            set(MYSQL_LIBRARY
                "${_AEROSENTINEL_MYSQL_CLIENT_LIBRARY}"
                CACHE FILEPATH "MySQL client library" FORCE)
        endif()

        unset(_AEROSENTINEL_MYSQL_SEARCH_PATHS)
        unset(_AEROSENTINEL_MYSQL_CLIENT_LIBRARY CACHE)
        unset(_AEROSENTINEL_MYSQL_CLIENT_LIBRARY_DIR)
    endif()
endif()
