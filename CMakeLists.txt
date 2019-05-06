cmake_minimum_required ( VERSION 2.8 )
project ( path_planning )

set ( CMAKE_BUILD_TYPE Release )
set ( CMAKE_CXX_STANDARD 11 )
set ( CMAKE_CXX_STANDARD_REQUIRED ON )
# Find Boost
find_package ( Boost COMPONENTS program_options filesystem REQUIRED )

# Find OpenCV
find_package( OpenCV REQUIRED )

include_directories ( ${Boost_INCLUDE_DIRS} )
include_directories ( include )

add_library( world src/world_generator.cpp )
add_library( path src/path_generator.cpp )
add_library( heuristic src/heuristic_generator.cpp )

add_executable ( astar src/main.cpp )

target_include_directories ( astar PRIVATE ${include} )
target_link_libraries ( astar  world path heuristic ${Boost_LIBRARIES} ${OpenCV_LIBS} )
