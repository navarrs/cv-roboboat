cmake_minimum_required( VERSION 3.1.0 )

project( CucoPerception )

# OpenGL 
find_package( OpenGL REQUIRED )

# Boost
find_package ( Boost COMPONENTS program_options filesystem REQUIRED )
include_directories ( ${Boost_INCLUDE_DIRS} )
include_directories ( include )

# Point cloud library
add_library ( pointcloud  ../src/pointcloud.cpp )
add_library ( groundseg   ../src/groundseg.cpp ) 

set(DEPENDENCIES realsense2 glfw ${OPENGL_LIBRARIES} pointcloud groundseg)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

# Ejecutable
add_executable( cuco ../src/cuco_main.cpp ../src/utils.hpp ../src/sensor_check.hpp )
target_link_libraries( cuco ${DEPENDENCIES} ${Boost_LIBRARIES} )
