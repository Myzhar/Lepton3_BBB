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
set(TRK_SRC ${CMAKE_CURRENT_SOURCE_DIR}/flir_tracker/src/flir_tracker.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/flir_tracker/src/Palettes.cpp )
set(ENC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/videoEncoder/src/videoEncoder.cpp )
#########################################################

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/videoEncoder/include
                     ${GSTREAMER_INCLUDE_DIRS}
                     ${GLIB_INCLUDE_DIRS}
                     /usr/lib/x86_64-linux-gnu/gstreamer-1.0/include/ # Modify with the correct folder for BeagleBone Blue
                     ${CMAKE_CURRENT_SOURCE_DIR}/flir_tracker/include
)

add_executable (test_tracker test_tracker.cpp ${TRK_SRC})
target_link_libraries (test_tracker LINK_PUBLIC
                       ${LIB_GSTREAMER}
                       opencv_core
                       opencv_highgui
                       opencv_imgproc)

add_executable (bbb_life_tracker bbb_life_tracker.cpp ${ENC_SRC} ${TRK_SRC} )
target_link_libraries (bbb_life_tracker LINK_PUBLIC
                       lepton3_grabber
                      ${LIB_GSTREAMER}
                       opencv_core
                       opencv_highgui
                       opencv_imgproc)


