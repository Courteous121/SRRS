find_package(OpenCV REQUIRED)

aux_source_directory(. RMVIDEOCAPTURE_DIRS)

add_library(RMVideoCapture ${RMVIDEOCAPTURE_DIRS})

target_include_directories(RMVideoCapture PUBLIC include)

target_link_libraries(RMVideoCapture ${OpenCV_LIBS} libMVSDK.so SerialPort)
target_link_libraries(RMVideoCapture Parameter)
