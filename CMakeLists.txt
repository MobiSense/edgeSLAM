cmake_minimum_required(VERSION 3.5)
project(edge_slam)

#set(CMAKE_CXX_STANDARD 11)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

#set(CMAKE_C_FLAGS_DEBUG "-O2 -g")
#set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g")

set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O1 -g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O1 -g")

set(CMAKE_C_FLAGS_RELEASE "-O1")
set(CMAKE_CXX_FLAGS_RELEASE "-O1")

LIST(APPEND CMAKE_C_FLAGS "-Wall -Wextra -DUSE_NVTX -Wno-deprecated-declarations -ftemplate-backtrace-limit=1")
LIST(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -DUSE_NVTX -Wno-deprecated-declarations -ftemplate-backtrace-limit=1")


find_package(CUDA QUIET REQUIRED)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
SET(CUDA_HOST_COMPILER /usr/bin/g++)
LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.1.0 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()


cuda_include_directories(
        ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
)

set(CLIENT "client")
set(CLIENT_LIBRARY "slam_client")
set(CLIENT_DIRECTORY "${PROJECT_SOURCE_DIR}/client")
set(SERVER "server")
set(SERVER_LIBRARY "slam_server")
set(SERVER_DIRECTORY "${PROJECT_SOURCE_DIR}/server")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

find_package(spdlog REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/debug/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/debug/bin)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin)


# CLIENT
#add_library(${CLIENT_LIBRARY} SHARED
cuda_add_library(${CLIENT_LIBRARY} SHARED
        ${CLIENT_DIRECTORY}/src/System.cc
        ${CLIENT_DIRECTORY}/src/Tracking.cc
        ${CLIENT_DIRECTORY}/src/LocalMapping.cc
        ${CLIENT_DIRECTORY}/src/LoopClosing.cc
        ${CLIENT_DIRECTORY}/src/ORBextractor.cc
        ${CLIENT_DIRECTORY}/src/ORBmatcher.cc
        ${CLIENT_DIRECTORY}/src/FrameDrawer.cc
        ${CLIENT_DIRECTORY}/src/Converter.cc
        ${CLIENT_DIRECTORY}/src/MapPoint.cc
        ${CLIENT_DIRECTORY}/src/KeyFrame.cc
        ${CLIENT_DIRECTORY}/src/Map.cc
        ${CLIENT_DIRECTORY}/src/MapDrawer.cc
        ${CLIENT_DIRECTORY}/src/Optimizer.cc
        ${CLIENT_DIRECTORY}/src/PnPsolver.cc
        ${CLIENT_DIRECTORY}/src/Frame.cc
        ${CLIENT_DIRECTORY}/src/KeyFrameDatabase.cc
        ${CLIENT_DIRECTORY}/src/MapElementUpdate.cc
        ${CLIENT_DIRECTORY}/src/Sim3Solver.cc
        ${CLIENT_DIRECTORY}/src/Initializer.cc
        ${CLIENT_DIRECTORY}/src/Viewer.cc
        ${CLIENT_DIRECTORY}/src/MapUpdater.cc
        ${CLIENT_DIRECTORY}/src/Client.cc
        ${CLIENT_DIRECTORY}/src/DataSetUtil.cc
        ${CLIENT_DIRECTORY}/src/Timer.cc
        ${CLIENT_DIRECTORY}/src/MapManager.cc
        ${CLIENT_DIRECTORY}/src/MapSlice.cc
        ${CLIENT_DIRECTORY}/src/cuda/Allocator_gpu.cu
        ${CLIENT_DIRECTORY}/src/cuda/Fast_gpu.cu
        ${CLIENT_DIRECTORY}/src/cuda/Orb_gpu.cu
        ${CLIENT_DIRECTORY}/src/cuda/Cuda.cu)

target_include_directories(${CLIENT_LIBRARY} PUBLIC
        ${CLIENT_DIRECTORY}
        ${CLIENT_DIRECTORY}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        )

target_link_libraries(${CLIENT_LIBRARY}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CLIENT_DIRECTORY}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${CLIENT_DIRECTORY}/Thirdparty/g2o/lib/libg2o.so
        nvToolsExt
        pthread
        ${Boost_LIBRARIES}
        boost_serialization
        opencv_cudabgsegm
        opencv_cudaobjdetect
        opencv_cudastereo
        opencv_cudafeatures2d
        #opencv_cudacodec
        opencv_cudaoptflow
        opencv_cudalegacy
        opencv_cudawarping
        opencv_cudaimgproc
        opencv_cudafilters
        spdlog::spdlog
        )

add_executable(${CLIENT}
        ${CLIENT_DIRECTORY}/Examples/Monocular/mono_euroc.cc)
#target_link_libraries(${CLIENT} ${CLIENT_LIBRARY} ${EDGE_GRPC_LIBRARY})
target_link_libraries(${CLIENT} ${CLIENT_LIBRARY})

add_executable(maploader
        ${CLIENT_DIRECTORY}/Examples/Monocular/map_loader.cc)
target_link_libraries(maploader ${CLIENT_LIBRARY})

add_executable(relocalizer
        ${CLIENT_DIRECTORY}/Examples/Monocular/relocalizer.cc)
target_link_libraries(relocalizer ${CLIENT_LIBRARY})

add_executable(mapoptimizer
        ${CLIENT_DIRECTORY}/Examples/Monocular/mapoptimizer.cc)
target_link_libraries(mapoptimizer ${CLIENT_LIBRARY})

add_executable(map_conveyance
        ${CLIENT_DIRECTORY}/Examples/Monocular/map_conveyance.cc)
target_link_libraries(map_conveyance ${CLIENT_LIBRARY})


# SERVER
if (FALSE)

    add_library(${SERVER_LIBRARY} SHARED
            ${SERVER_DIRECTORY}/src/System.cc
            ${SERVER_DIRECTORY}/src/Tracking.cc
            ${SERVER_DIRECTORY}/src/LocalMapping.cc
            ${SERVER_DIRECTORY}/src/LoopClosing.cc
            ${SERVER_DIRECTORY}/src/ORBextractor.cc
            ${SERVER_DIRECTORY}/src/ORBmatcher.cc
            ${SERVER_DIRECTORY}/src/FrameDrawer.cc
            ${SERVER_DIRECTORY}/src/Converter.cc
            ${SERVER_DIRECTORY}/src/MapPoint.cc
            ${SERVER_DIRECTORY}/src/KeyFrame.cc
            ${SERVER_DIRECTORY}/src/Map.cc
            ${SERVER_DIRECTORY}/src/MapDrawer.cc
            ${SERVER_DIRECTORY}/src/Optimizer.cc
            ${SERVER_DIRECTORY}/src/PnPsolver.cc
            ${SERVER_DIRECTORY}/src/Frame.cc
            ${SERVER_DIRECTORY}/src/KeyFrameDatabase.cc
            ${SERVER_DIRECTORY}/src/Sim3Solver.cc
            ${SERVER_DIRECTORY}/src/Initializer.cc
            ${SERVER_DIRECTORY}/src/Viewer.cc
            ${SERVER_DIRECTORY}/src/GlobalInstance.cpp)

    target_include_directories(${SERVER_LIBRARY} PUBLIC
            ${SERVER_DIRECTORY}
            ${PROJECT_SOURCE_DIR}/grpc
            ${SERVER_DIRECTORY}/include
            ${EIGEN3_INCLUDE_DIR}
            ${Pangolin_INCLUDE_DIRS})

    target_link_libraries(${SERVER_LIBRARY}
            ${OpenCV_LIBS}
            ${EIGEN3_LIBS}
            ${Pangolin_LIBRARIES}
            ${SERVER_DIRECTORY}/Thirdparty/DBoW2/lib/libDBoW2.so
            ${SERVER_DIRECTORY}/Thirdparty/g2o/lib/libg2o.so
            gRPC::grpc++_reflection
            protobuf::libprotobuf
            ${Boost_LIBRARIES}
            boost_serialization
            )

    add_executable(${SERVER}
            ${SERVER_DIRECTORY}/Examples/Monocular/mono_euroc.cc)
    target_link_libraries(${SERVER} ${SERVER_LIBRARY} ${EDGE_GRPC_LIBRARY})
endif ()
