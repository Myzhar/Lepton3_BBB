SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

find_package(OpenCV REQUIRED)

##############################################################################
# GStreamer
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package( GStreamer-1.0 REQUIRED )
find_package( GLIB REQUIRED )
set(LIB_GSTREAMER ${GSTREAMER_LIBRARIES} ${GSTREAMER_APP_LIBRARIES}
    -lgobject-2.0 ${GLIB_LIBRARIES}
)
##############################################################################

#########################################################
# SOURCES
file(GLOB_RECURSE DEMO_SRC *.c *.cpp)
# HEADERS
file(GLOB_RECURSE DEMO_HDR *.h *.hpp)
#########################################################

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/videoEncoder/include
                     ${GSTREAMER_INCLUDE_DIRS}
                     ${GLIB_INCLUDE_DIRS}
                     /usr/lib/x86_64-linux-gnu/gstreamer-1.0/include/ # Modify with the correct folder for BeagleBone Blue
)


add_executable (gstreamer_demo ${DEMO_SRC} )
target_link_libraries (gstreamer_demo LINK_PUBLIC 
                       lepton3_grabber 
                       ${LIB_GSTREAMER}
                       opencv_core 
                       opencv_highgui 
                       opencv_imgproc)
