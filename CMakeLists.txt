cmake_minimum_required(VERSION 2.8)
project(lsd_slam)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/lsd_slam_core/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o
find_package(X11 REQUIRED)
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
find_package(GFlags REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -march=native -std=c++11"
)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast -DDEBUG=OFF -DNDEBUG")

set(LSD_SLAM_SOURCE
   lsd_slam_core/src/DataStructures/Frame.cpp
   lsd_slam_core/src/DataStructures/FramePoseStruct.cpp
   lsd_slam_core/src/DataStructures/FrameMemory.cpp
   lsd_slam_core/src/SlamSystem.cpp
   lsd_slam_core/src/LiveSLAMWrapper.cpp
   lsd_slam_core/src/DepthEstimation/DepthMap.cpp
   lsd_slam_core/src/DepthEstimation/DepthMapPixelHypothesis.cpp
   lsd_slam_core/src/util/globalFuncs.cpp
   lsd_slam_core/src/util/SophusUtil.cpp
   lsd_slam_core/src/util/settings.cpp
   lsd_slam_core/src/util/Undistorter.cpp
   lsd_slam_core/src/Tracking/Sim3Tracker.cpp
   lsd_slam_core/src/Tracking/Relocalizer.cpp
   lsd_slam_core/src/Tracking/SE3Tracker.cpp
   lsd_slam_core/src/Tracking/TrackingReference.cpp
   lsd_slam_core/src/IOWrapper/Timestamp.cpp
   lsd_slam_core/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
   lsd_slam_core/src/GlobalMapping/FabMap.cpp
   lsd_slam_core/src/GlobalMapping/KeyFrameGraph.cpp
   lsd_slam_core/src/GlobalMapping/g2oTypeSim3Sophus.cpp
   lsd_slam_core/src/GlobalMapping/TrackableKeyFrameSearch.cpp
)

include_directories(
   ${EIGEN3_INCLUDE_DIR}
   ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
   ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
   ${OpenCV_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS}
   ${GFLAGS_INCLUDE_DIRS}
   ${G2O_INCLUDE_DIR}
   lsd_slam_core/src/
   lsd_slam_core/thirdparty/Sophus/
)

add_library(lsdslam ${LSD_SLAM_SOURCE})

target_link_libraries(lsdslam ${G2O_LIBS} ${CSPARSE_LIBRARIES}
    ${Boost_LIBRARIES} ${X11_LIBRARIES} ${OpenCV_LIBS} ${GFLAGS_LIBRARIES} pthread glog)

add_executable(main_lsd_slam main_lsd_slam.cpp)
target_link_libraries(main_lsd_slam lsdslam)
