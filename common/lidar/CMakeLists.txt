find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads)
find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations -Wno-deprecated-declarations ")

# ==== Libraries ====
aux_source_directory(./src DIR_LIDAR)
add_library (Lidar ${DIR_LIDAR})
target_link_libraries(Lidar
  PUBLIC Eigen3::Eigen
  PUBLIC jsoncpp_lib)
target_link_libraries(Lidar
  PUBLIC Threads::Threads glfw GLEW::GLEW OpenGL::GL)
target_link_libraries(Lidar PUBLIC ${OpenCV_LIBS})
target_include_directories(Lidar PUBLIC include)
target_link_libraries(Lidar PUBLIC Parameter)



