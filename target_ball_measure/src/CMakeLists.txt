aux_source_directory (. SRC_LIST)

find_package(PCL 1.5 REQUIRED)

find_library(Bayesopt_LIBRARIES NAMES bayesopt)
find_path(Bayesopt_INCLUDE_PATH NAMES bayesopt/bayesopt.h)

find_library(NLOPT_LIBRARIES NAMES nlopt)
find_path(NLOPT_INCLUDE_PATH NAMES nlopt.h)

include_directories(../include
					${PCL_INCLUDE_DIRS}
					${Bayesopt_INCLUDE_PATH}
					${NLOPT_INCLUDE_PATH}
					)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (target_ball_measure ${SRC_LIST})
target_link_libraries (target_ball_measure 
					   ${PCL_LIBRARIES}
					   ${Bayesopt_LIBRARIES}
					   ${NLOPT_LIBRARIES})

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)