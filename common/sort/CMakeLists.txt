aux_source_directory(./src DIR_SORT)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library (Sort ${DIR_SORT})
target_include_directories(Sort PUBLIC include)
target_link_libraries(Sort Parameter)
target_link_libraries(Sort ${OpenCV_LIBS})
