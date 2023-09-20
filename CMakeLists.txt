cmake_minimum_required (VERSION 3.10)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
# 對了！網這裏看！更換設備後需要把下面空格右邊的TensorRT路徑修改一下哦
set(TensorRT_DIR /home/robotlab/Documents/TensorRT-7.2.3.4/)
project (RadarStation)

if (POLICY CMP0072)
  set(OpenGL_GL_PREFERENCE LEGACY)
endif()

# 激光雷達損壞處理
option(LIDAR_DAMAGE "lidar has damages" OFF)
if(LIDAR_DAMAGE)
    add_definitions(-DLIDAR_DAMAGE)
    message(AUTHOR_WARNING "8万的雷達無了，還好我爲你保留了備用定位方案！")
endif()

# 飛鏢相機處理
option(DART_CORRECT "short the dart camera time" ON)
if(DART_CORRECT)
    add_definitions(-DDART_CORRECT)
    message(AUTHOR_WARNING "飛鏢開倉時才錄製！減少錄製文件負擔！")
endif()

# cuda與cudnn
find_package(CUDA REQUIRED)
include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include)
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# TensorRT處理
if(EXISTS ${TensorRT_DIR})
    message(STATUS "Found TensorRT: ${TensorRT_DIR}")
else()
    message(FATAL_ERROR "use你的mind think think 是不是TensorRT的路徑沒改啊？！")
endif()
include_directories(${TensorRT_DIR}/include)
link_directories(${TensorRT_DIR}/lib)

add_subdirectory(modules)
add_subdirectory(common)

add_executable(RadarStation main.cpp)
target_link_libraries(RadarStation Radar)
target_link_libraries(RadarStation Parameter)
target_link_libraries(RadarStation CommandParser)
