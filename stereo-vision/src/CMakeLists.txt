cmake_minimum_required(VERSION 2.8)
project(Vanttec)

set(CMAKE_CXX_STANDARD 11) 

find_package(OpenCV REQUIRED)



add_executable(stereo_match src/stereo_match.cpp)
target_link_libraries(stereo_match ${OpenCV_LIBS})

add_executable(image_cap src/image_cap.cpp)
target_link_libraries(image_cap ${OpenCV_LIBS})

add_executable(imagelist_creator src/imagelist_creator.cpp)
target_link_libraries(imagelist_creator ${OpenCV_LIBS})

add_executable(stereo_calib src/stereo_calib.cpp)
target_link_libraries(stereo_calib ${OpenCV_LIBS})
