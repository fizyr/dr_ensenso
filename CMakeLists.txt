cmake_minimum_required(VERSION 3.8)
project(dr_ensenso)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)
add_link_options(LINKER:--as-needed)

find_package(catkin REQUIRED COMPONENTS
	estd
	dr_util
)

# Check if JsonCpp target does not exist and then create it manually
if(NOT TARGET JsonCpp::JsonCpp)
  # Attempt to find JsonCpp library - modify paths as needed
  find_library(JSONCPP_LIBRARY NAMES jsoncpp PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/lib /usr/local/lib)
  find_path(JSONCPP_INCLUDE_DIR NAMES json/json.h PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/include /usr/local/include)

  # Create an imported target
  add_library(JsonCpp::JsonCpp SHARED IMPORTED)
  set_target_properties(JsonCpp::JsonCpp PROPERTIES
    IMPORTED_LOCATION "${JSONCPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${JSONCPP_INCLUDE_DIR}")
endif()

find_package(Ensenso REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem date_time chrono)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(fmt REQUIRED)

find_package(PCL REQUIRED)

# Remove python2 dependencies, we don't need these anyway.
list(REMOVE_ITEM PCL_LIBRARIES
	"vtkGUISupportQt"
	"vtkGUISupportQtOpenGL"
	"vtkGUISupportQtSQL"
	"vtkGUISupportQtWebkit"
	"vtkViewsQt"
	"vtkRenderingQt"
)
list(FILTER PCL_LIBRARIES    EXCLUDE REGEX "(^|/)(libNxLib|libpython2).*[.]so$")
list(FILTER catkin_LIBRARIES EXCLUDE REGEX "(^|/)(libpython2).*[.]so$")


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	DEPENDS PCL Ensenso Boost EIGEN3 OpenCV jsoncpp
	CATKIN_DEPENDS estd
	CFG_EXTRAS "compile_options.cmake"
)

include_directories(SYSTEM
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${Ensenso_INCLUDE_DIRS}
	${jsoncpp_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
include_directories(include/${PROJECT_NAME})

add_library(${PROJECT_NAME}
	src/eigen.cpp
	src/ensenso.cpp
	src/opencv.cpp
	src/pcl.cpp
	src/util.cpp
)

target_link_libraries(${PROJECT_NAME}
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	${EIGEN3_LIBRARIES}
	${Ensenso_LIBRARIES}
	${jsoncpp_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	${fmt_LIBRARIES}
	fmt::fmt
)

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
