cmake_minimum_required(VERSION 3.8)
project(dr_ensenso)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_ros
	pcl_conversions
)

find_package(Ensenso REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem date_time chrono)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

SET(SYSTEM_LIBRARIES    ${Ensenso_LIBRARIES}    ${Boost_LIBRARIES}    ${EIGEN3_LIBRARIES}    ${OpenCV_LIBRARIES})
SET(SYSTEM_INCLUDE_DIRS ${Ensenso_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	DEPENDS Ensenso Boost EIGEN3 OpenCV
	CATKIN_DEPENDS pcl_ros
)

include_directories(SYSTEM
	${catkin_INCLUDE_DIRS}
	${SYSTEM_INCLUDE_DIRS}
)
include_directories(include/${PROJECT_NAME})

add_library(${PROJECT_NAME}
	src/eigen.cpp
	src/ensenso.cpp
	src/error.cpp
	src/opencv.cpp
	src/pcl.cpp
	src/util.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SYSTEM_LIBRARIES})

install(
	TARGETS "${PROJECT_NAME}"
	ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
	RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
	DIRECTORY "include/${PROJECT_NAME}/"
	DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
