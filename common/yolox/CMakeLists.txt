aux_source_directory(./src DIR_YOLOX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library (YOLOX ${DIR_YOLOX})
target_link_libraries(YOLOX nvinfer)
target_include_directories(YOLOX PUBLIC include)
target_link_libraries(YOLOX cudart)
target_link_libraries(YOLOX Parameter)
target_link_libraries(YOLOX ${OpenCV_LIBS})

add_definitions(-O2 -pthread)

