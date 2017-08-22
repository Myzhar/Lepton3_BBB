#########################################################
# SOURCES
file(GLOB_RECURSE GRB_SRC *.c *.cpp)
# HEADERS
file(GLOB_RECURSE GRB_HDR *.h *.hpp)
#########################################################

add_library (lepton3_grabber ${GRB_SRC})

# Make sure the compiler can find include files for our Hello library
# when other libraries or executables link to Hello
target_include_directories (lepton3_grabber PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
