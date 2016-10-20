cmake_minimum_required(VERSION 2.8.3)
project(dr_ensenso)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dr_base
	pcl_ros
	pcl_conversions
	cmake_modules
)

find_package(Ensenso REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem date_time chrono)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

SET(SYSTEM_LIBRARIES    ${Ensenso_LIBRARIES}    ${Boost_LIBRARIES}    ${EIGEN3_LIBRARIES}    ${OpenCV_LIBRARIES}    ${PCL_LIBRARIES})
SET(SYSTEM_INCLUDE_DIRS ${Ensenso_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include ${SYSTEM_INCLUDE_DIRS}
	LIBRARIES ${PROJECT_NAME} ${SYSTEM_LIBRARIES}
	CATKIN_DEPENDS
)

dr_include_directories(
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
	${SYSTEM_INCLUDE_DIRS}
)

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
