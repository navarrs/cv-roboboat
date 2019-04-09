cmake_minimum_required ( VERSION 2.8 )
project ( path_planning )

set ( CMAKE_BUILD_TYPE Release )

# Find Boost
find_package ( Boost COMPONENTS program_options filesystem REQUIRED )

# Find OpenCV
find_package( OpenCV REQUIRED )

include_directories ( ${Boost_INCLUDE_DIRS} )
include_directories ( include )

add_executable ( aco src/main.cpp )

target_include_directories ( aco PRIVATE ${include} )

target_link_libraries ( aco ${Boost_LIBRARIES} ${OpenCV_LIBS} )
