find_package(Threads REQUIRED)

aux_source_directory(. COMMAND_PARSER_DIRS)

add_library(CommandParser ${COMMAND_PARSER_DIRS})

target_include_directories(CommandParser PUBLIC include)

target_link_libraries(CommandParser ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(CommandParser Radar)
target_link_libraries(CommandParser Calibrator)
target_link_libraries(CommandParser StorageMat)
