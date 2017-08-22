cmake_minimum_required (VERSION 2.8.11)
project (lepton3_grabber)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)
set(CMAKE_DEBUG_POSTFIX "_d")

#########################################################
# FLAGS
if (UNIX)
    ADD_DEFINITIONS(
        -std=c++11
    )
endif (UNIX)
#########################################################

# Library source code
add_subdirectory (grabber_lib)

# GStreamer demo
add_subdirectory (opencv_demo)
