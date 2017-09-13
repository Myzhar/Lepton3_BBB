SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

#########################################################
# SOURCES
file(GLOB_RECURSE GRB_SRC *.c *.cpp)
# HEADERS
file(GLOB_RECURSE GRB_HDR *.h *.hpp)
#########################################################

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Lepton_SDK/include)

add_library( lepton3_grabber ${GRB_SRC} )
target_link_libraries( lepton3_grabber
                       ${CMAKE_CURRENT_SOURCE_DIR}/Lepton_SDK/lib/LEPTON_SDK.a
                       -ldl
)

# Make sure the compiler can find include files for our Hello library
# when other libraries or executables link to Hello
target_include_directories( lepton3_grabber PUBLIC
                            ${CMAKE_CURRENT_SOURCE_DIR}/include
                            ${CMAKE_CURRENT_SOURCE_DIR}/Lepton_SDK/include
)
