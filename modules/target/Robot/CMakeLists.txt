# 查找src文件夹下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
aux_source_directory(./src DIR_ROBOT)

#关于opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 生成链接库，使用上方的变量
add_library (Robot ${DIR_ROBOT})

#将include里的头文件加进上方的链接库中
target_include_directories(Robot PUBLIC include)
#opencv链接
target_link_libraries(Robot ${OpenCV_LIBS})
