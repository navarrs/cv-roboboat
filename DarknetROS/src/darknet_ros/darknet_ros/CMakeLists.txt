cmake_minimum_required(VERSION 2.8)
project(darknet_ros)

#---------------------------------------------------------------------------#
# Set colors
string(ASCII 27 Esc)
set(Red         "${Esc}[31m")
set(Green       "${Esc}[32m")
set(Yellow      "${Esc}[33m")
set(Blue        "${Esc}[34m")
set(Magenta     "${Esc}[35m")
set(Cyan        "${Esc}[36m")
set(White       "${Esc}[37m")
set(BoldRed     "${Esc}[1;31m")
set(BoldGreen   "${Esc}[1;32m")
set(BoldYellow  "${Esc}[1;33m")
set(BoldBlue    "${Esc}[1;34m")
set(BoldMagenta "${Esc}[1;35m")
set(BoldCyan    "${Esc}[1;36m")
set(BoldWhite   "${Esc}[1;37m")

#---------------------------------------------------------------------------#
# Set c++11 cmake flags
set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )

#---------------------------------------------------------------------------#
# Define path for darknet 
message( " ${Blue}[START] Searching darknet path..." )
find_path( DARKNET_PATH
         NAMES "README.md"
         HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../darknet/" )
if (NOT DARKNET_PATH)
  message( " ${Red}[FAILED] Darknet path not found. " )
else()
  message( " ${Yellow}-----Darknet path dir = ${DARKNET_PATH}" )
  add_definitions( -DDARKNET_FILE_PATH="${DARKNET_PATH}" )
  message( " ${Cyan}[DONE]" )
endif()

#--------------------------------------------------------------------------#
# Find CUDA
message( " ${Blue}[START] Searching for CUDA..." )
find_package( CUDA )
if ( CUDA_FOUND )
  find_package( CUDA REQUIRED )
  message( STATUS "${Yellow}----- CUDA Version: ${CUDA_VERSION_STRINGS}" )
  message( STATUS "${Yellow}----- CUDA Libraries: ${CUDA_LIBRARIES}" )
  set( CUDA_NVCC_FLAGS
       ${CUDA_NVCC_FLAGS};
         -O3
         -gencode arch=compute_30,code=sm_30
         -gencode arch=compute_35,code=sm_35
         -gencode arch=compute_50,code=[sm_50,compute_50]
         -gencode arch=compute_52,code=[sm_52,compute_52]
         -gencode arch=compute_61,code=sm_61
         -gencode arch=compute_62,code=sm_62
    )
    add_definitions( -DGPU )
    message( " ${Cyan}[DONE]" )
else()
  message( " ${Red}[FAILED] CUDA not found." )
  list(APPEND LIBRARIIES "m")
endif()


#--------------------------------------------------------------------------#
# Find X11 - Communication protocol that allows apps to use drawing primitives
message( " ${Blue}[START] Searching for X11..." )
find_package( X11 REQUIRED )
if ( X11_FOUND )
  include_directories( ${X11_INCLUDE_DIR} )
  link_libraries( ${X11_LIBRARIES} )
  message( STATUS "${Yellow}----- X11 path: ${X11_INCLUDE_DIR}" )
  message( STATUS "${Yellow}----- X11 libraries: ${X11_LIBRARIES}" )
  message( " ${Cyan}[DONE]" )
else()
  message( " ${Red}[FAILED] X11 not found." )
endif()

#--------------------------------------------------------------------------#
# Find other required packages
message( " ${Blue}[START] Searching for Boost..." )
find_package(Boost REQUIRED COMPONENTS thread)
if ( Boost_FOUND )
  message( " ${Cyan}[DONE]" )
else()
  message( " ${Red}[FAILED] Boost not found.")
endif()

message( " ${Blue}[START] Searching for OpenCV..." )
find_package( OpenCV REQUIRED )
if ( OpenCV_FOUND )
  include_directories(${OpenCV_INCLUDE_DIRS})
  message( STATUS "${Yellow}----- OpenCV dirs: ${OpenCV_INCLUDE_DIRS}" )
  message( STATUS "${Yellow}----- Enable OpenCV for darknet..." )
  add_definitions( -DOPENCV )
  add_definitions( -O4 -g )
  message( " ${Cyan}[DONE]" )
else()
  message( " ${Red}[FAILED] OpenCV not found.")
endif()

message( " ${Blue}[START] Searching for catkin..." )
find_package( catkin REQUIRED 
              COMPONENTS 
                cv_bridge 
                roscpp 
                rospy 
                std_msgs 
                actionlib 
                darknet_ros_msgs 
                image_transport 
            )
if ( catkin_FOUND )
  message( " ${Cyan}[DONE]" )
else()
  message( " ${Red}[FAILED] catkin not found.")
endif()

message( " ${Blue}[START] Catkin configuration..." )
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    actionlib
    rospy
    std_msgs
    darknet_ros_msgs
    image_transport
  DEPENDS
    Boost
)
message( " ${Cyan}[DONE]" )

#--------------------------------------------------------------------------#
# Build
message( " ${Blue}[START] Building project..." )
message( " ${Green} Include directories..." )
include_directories(
  ${DARKNET_PATH}/src
  ${DARKNET_PATH}/include
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

message( " ${Green} Link libraries..." )
message( " ${Yellow}-----Using CUDA..." )
if (CUDA_FOUND)

  message( " ${Magenta}-----------YES." )
  message( " ${Yellow}-----Link CUDA directories." )
  link_directories( /usr/local/cuda/lib64 )

  cuda_add_library(${PROJECT_NAME}_lib
    src/YoloObjectDetector.cpp
    src/image_interface.c

    ${DARKNET_PATH}/src/activation_layer.c        
    ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/activations.c             
    ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/avgpool_layer.c           
    ${DARKNET_PATH}/src/layer.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         
    ${DARKNET_PATH}/src/list.c
    ${DARKNET_PATH}/src/blas.c                    
    ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/box.c                     
    ${DARKNET_PATH}/src/lstm_layer.c
    ${DARKNET_PATH}/src/col2im.c                  
    ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/connected_layer.c         
    ${DARKNET_PATH}/src/maxpool_layer.c
    ${DARKNET_PATH}/src/convolutional_layer.c     
    ${DARKNET_PATH}/src/network.c
    ${DARKNET_PATH}/src/cost_layer.c              
    ${DARKNET_PATH}/src/normalization_layer.c
    ${DARKNET_PATH}/src/crnn_layer.c              
    ${DARKNET_PATH}/src/option_list.c
    ${DARKNET_PATH}/src/crop_layer.c              
    ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/cuda.c                    
    ${DARKNET_PATH}/src/region_layer.c
    ${DARKNET_PATH}/src/data.c                    
    ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/deconvolutional_layer.c   
    ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/demo.c                    
    ${DARKNET_PATH}/src/route_layer.c
    ${DARKNET_PATH}/src/detection_layer.c         
    ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/dropout_layer.c           
    ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/gemm.c                    
    ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/gru_layer.c               
    ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/upsample_layer.c          
    ${DARKNET_PATH}/src/logistic_layer.c
    ${DARKNET_PATH}/src/l2norm_layer.c            
    ${DARKNET_PATH}/src/yolo_layer.c

    ${DARKNET_PATH}/examples/art.c                
    ${DARKNET_PATH}/examples/lsd.c
    ${DARKNET_PATH}/examples/attention.c          
    ${DARKNET_PATH}/examples/nightmare.c
    ${DARKNET_PATH}/examples/captcha.c            
    ${DARKNET_PATH}/examples/regressor.c
    ${DARKNET_PATH}/examples/cifar.c              
    ${DARKNET_PATH}/examples/rnn.c
    ${DARKNET_PATH}/examples/classifier.c         
    ${DARKNET_PATH}/examples/segmenter.c
    ${DARKNET_PATH}/examples/coco.c               
    ${DARKNET_PATH}/examples/super.c
    ${DARKNET_PATH}/examples/darknet.c            
    ${DARKNET_PATH}/examples/tag.c
    ${DARKNET_PATH}/examples/detector.c           
    ${DARKNET_PATH}/examples/yolo.c
    ${DARKNET_PATH}/examples/go.c

    ${DARKNET_PATH}/src/activation_kernels.cu     
    ${DARKNET_PATH}/src/crop_layer_kernels.cu
    ${DARKNET_PATH}/src/avgpool_layer_kernels.cu  
    ${DARKNET_PATH}/src/deconvolutional_kernels.cu
    ${DARKNET_PATH}/src/blas_kernels.cu           
    ${DARKNET_PATH}/src/dropout_layer_kernels.cu
    ${DARKNET_PATH}/src/col2im_kernels.cu         
    ${DARKNET_PATH}/src/im2col_kernels.cu
    ${DARKNET_PATH}/src/convolutional_kernels.cu  
    ${DARKNET_PATH}/src/maxpool_layer_kernels.cu
  )

  message( " ${Yellow}-----Link target libraries to project." )
  target_link_libraries(${PROJECT_NAME}_lib
    m
    pthread
    stdc++
    cuda
    cudart
    cublas
    curand
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

  message( " ${Yellow}-----Add CUDA executable." )
  cuda_add_executable(${PROJECT_NAME}
    src/yolo_object_detector_node.cpp
  )
else()
  message( " ${Magenta}----------NO." )
  message( " ${Yellow}-----Add project library." )
  add_library(${PROJECT_NAME}_lib
    src/YoloObjectDetector.cpp
    src/image_interface.c

    ${DARKNET_PATH}/src/activation_layer.c        
    ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/activations.c             
    ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/avgpool_layer.c           
    ${DARKNET_PATH}/src/layer.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         
    ${DARKNET_PATH}/src/list.c
    ${DARKNET_PATH}/src/blas.c                    
    ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/box.c                     
    ${DARKNET_PATH}/src/lstm_layer.c
    ${DARKNET_PATH}/src/col2im.c                  
    ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/connected_layer.c         
    ${DARKNET_PATH}/src/maxpool_layer.c
    ${DARKNET_PATH}/src/convolutional_layer.c     
    ${DARKNET_PATH}/src/network.c
    ${DARKNET_PATH}/src/cost_layer.c              
    ${DARKNET_PATH}/src/normalization_layer.c
    ${DARKNET_PATH}/src/crnn_layer.c              
    ${DARKNET_PATH}/src/option_list.c
    ${DARKNET_PATH}/src/crop_layer.c              
    ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/cuda.c                    
    ${DARKNET_PATH}/src/region_layer.c
    ${DARKNET_PATH}/src/data.c                    
    ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/deconvolutional_layer.c   
    ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/demo.c                    
    ${DARKNET_PATH}/src/route_layer.c
    ${DARKNET_PATH}/src/detection_layer.c         
    ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/dropout_layer.c           
    ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/gemm.c                    
    ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/gru_layer.c               
    ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/upsample_layer.c          
    ${DARKNET_PATH}/src/logistic_layer.c
    ${DARKNET_PATH}/src/l2norm_layer.c            
    ${DARKNET_PATH}/src/yolo_layer.c

    ${DARKNET_PATH}/examples/art.c                
    ${DARKNET_PATH}/examples/lsd.c
    ${DARKNET_PATH}/examples/attention.c          
    ${DARKNET_PATH}/examples/nightmare.c
    ${DARKNET_PATH}/examples/captcha.c            
    ${DARKNET_PATH}/examples/regressor.c
    ${DARKNET_PATH}/examples/cifar.c              
    ${DARKNET_PATH}/examples/rnn.c
    ${DARKNET_PATH}/examples/classifier.c         
    ${DARKNET_PATH}/examples/segmenter.c
    ${DARKNET_PATH}/examples/coco.c               
    ${DARKNET_PATH}/examples/super.c
    ${DARKNET_PATH}/examples/darknet.c            
    ${DARKNET_PATH}/examples/tag.c
    ${DARKNET_PATH}/examples/detector.c           
    ${DARKNET_PATH}/examples/yolo.c
    ${DARKNET_PATH}/examples/go.c
  )

  message( " ${Yellow}-----Link target libraries to project." )
  target_link_libraries(${PROJECT_NAME}_lib
    m
    pthread
    stdc++
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )

  message( " ${Yellow}-----Add executable." )
  add_executable(${PROJECT_NAME}
    src/yolo_object_detector_node.cpp
  )
endif() 

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

add_dependencies(${PROJECT_NAME}_lib
  darknet_ros_msgs_generate_messages_cpp
)
message( " ${Cyan}[DONE]" ) 

#--------------------------------------------------------------------------#
# Install
message( " ${Blue}[START] Installing..." ) 
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
message( " ${Cyan}[DONE]" )

#--------------------------------------------------------------------------#
# Download yolov2-tiny.weights
set( PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights" )
set( FILE "${PATH}/yolov2-tiny.weights" )
message( STATUS " ${Blue}[START] Checking and downloading yolov2-tiny.weights if needed ..." )
if ( NOT EXISTS "${FILE}" )
  message( " ${Magenta}-----Downloading yolov2-tiny.weights..." ) 
  message( STATUS "... file does not exist. Downloading now ..." )
  execute_process( COMMAND wget http://pjreddie.com/media/files/yolov2-tiny.weights -P ${PATH} )
  message( " ${Cyan}[DONE]" )
else()
  message( " ${Magenta}-----Already downloaded yolov2-tiny.weights." ) 
endif()

#--------------------------------------------------------------------------#
# Download yolov3.weights
set( FILE "${PATH}/yolov3.weights" )
message( STATUS "Checking and downloading yolov3.weights if needed ..." )
if ( NOT EXISTS "${FILE}" )
  message( " ${Magenta}------Downloading yolov3weights..." )
  message( STATUS "... file does not exist. Downloading now ..." )
  execute_process( COMMAND wget http://pjreddie.com/media/files/yolov3.weights -P ${PATH} )
  message( " ${Cyan}[DONE]" )
else()
  message( " ${Magenta}-----Already downloaded yolov3.weights." )
endif()

#--------------------------------------------------------------------------#
# Testing 
message( STATUS " ${Blue}[START] Run test..." )
if(CATKIN_ENABLE_TESTING)
  # Download yolov2.weights
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
  set(FILE "${PATH}/yolov2.weights")
  message(STATUS "Checking and downloading yolov2.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget http://pjreddie.com/media/files/yolov2.weights -P ${PATH})
  endif()

  find_package(rostest REQUIRED)

  # Object detection in images.
  add_rostest_gtest(${PROJECT_NAME}_object_detection-test
    test/object_detection.test
    test/test_main.cpp
    test/ObjectDetection.cpp
  )
  target_link_libraries(${PROJECT_NAME}_object_detection-test
    ${catkin_LIBRARIES}
  )
endif()
message( " ${Cyan}[DONE]" )