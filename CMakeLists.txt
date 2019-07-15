cmake_minimum_required(VERSION 2.8.3)
project(ddd_wall_mapping)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -fPIC -Wno-sign-compare -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

add_subdirectory(src/third_parties/libpointmatcher)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
#find_package(glog)
find_package(Threads REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen ${CERES_EIGEN_VERSION} EXACT QUIET)

SET(CUDA_INCLUDE_DIRS /usr/local/cuda-10.0/include)
find_package(CUDA)
if (CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
endif ()


find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            tf
            visualization_msgs
            pcl_conversions
            pcl_ros
            sensor_msgs
            message_filters
            ${PCL_OPENMP_PACKAGES}
        )

option(USE_OPENMP "Enable OpenMP?" ON) # set to OFF to disable
if(USE_OPENMP)
 FIND_PACKAGE(OpenMP)
 if(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP flags = ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 else(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP not supported")
 endif(OPENMP_FOUND OR OpenMP_FOUND)
endif(USE_OPENMP)


catkin_package(
    CATKIN_DEPENDS
        roscpp
        tf
        pcl_ros
        pcl_conversions
        visualization_msgs 
        std_msgs 
        sensor_msgs
        #message_filters
        ${PCL_OPENMP_PACKAGES}
   LIBRARIES slam_sensors
)


include_directories(
  include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIR})
#SET(CUDA_INCLUDE_DIRS /usr/local/cuda-10.0/include)
include_directories(
  src
  src/third_parties
  ${CUDA_INCLUDE_DIRS}
)

MESSAGE(STATUS "CUDA_INCLUDE_DIRS: " ${CUDA_INCLUDE_DIRS})
MESSAGE(STATUS "catkin_INCLUDE_DIRS: " ${catkin_INCLUDE_DIRS})

add_library(slam_sensor
                    src/imu.cc
                    src/my_imu.cc
                    src/lidar.cc
                    src/msg_conversion.cc
                    src/sensors.cc
                    src/options.cc
                    src/common/macro_defines.cc
                    src/common/pugixml.cc
                    src/extr_PlaneFeature.cc
                    )


add_executable(imu_preintegration 
  src/imu_preintegration.cc
)
add_dependencies(imu_preintegration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_preintegration
    slam_sensor
    ${catkin_LIBRARIES}
    pointmatcher
    gtsam
    ceres
)


add_executable(ndtMap
  src/ndtMap.cc
)
add_dependencies(ndtMap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ndtMap
    slam_sensor
    ${catkin_LIBRARIES}
    pointmatcher
    gtsam
    ceres
)

add_executable(msgs_synchronizer src/msgs_synchronizer.cc)
target_link_libraries(msgs_synchronizer ${catkin_LIBRARIES})

MESSAGE(STATUS "catkin_LIBRARIES: " ${catkin_LIBRARIES})

install( TARGETS
    slam_sensor
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

