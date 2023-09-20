# 查找src文件夹下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
aux_source_directory(./src DIR_RADAR)

find_package(Threads REQUIRED)
#关于opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 生成链接库，使用上方的变量
add_library (Radar ${DIR_RADAR})

#将include里的头文件加进上方的链接库中
target_include_directories(Radar PUBLIC include)
#opencv链接
target_link_libraries(Radar ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(Radar ${OpenCV_LIBS})
target_link_libraries(Radar Detector)
target_link_libraries(Radar Locator)
target_link_libraries(Radar RMVideoCapture)
target_link_libraries(Radar StorageMat)
target_link_libraries(Radar Referee)
target_link_libraries(Radar Lidar)
