find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

cmake_minimum_required (VERSION 3.10)

add_library(SerialPort STATIC SerialPort.cpp)

target_include_directories(SerialPort PUBLIC include)

target_link_libraries(SerialPort ${OpenCV_LIBS})
