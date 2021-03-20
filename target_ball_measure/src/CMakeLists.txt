aux_source_directory (. SRC_LIST)

include_directories (../include)
find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (target_ball_measure ${SRC_LIST})
target_link_libraries (target_ball_measure ${PCL_LIBRARIES})

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)