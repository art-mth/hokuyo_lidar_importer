cmake_minimum_required(VERSION 2.8.12)

set(SOURCES
    "src/hokuyo_lidar_importer.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/hokuyo_lidar_importer.h"
)

#set compiler flags
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"))
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wreturn-type -Wpedantic ")
endif()

include_directories("include")

add_library(hokuyo_lidar_importer MODULE ${SOURCES} ${HEADERS})
target_link_libraries(hokuyo_lidar_importer PRIVATE lmscore urg_cpp)
