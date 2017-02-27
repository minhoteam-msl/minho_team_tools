cmake_minimum_required(VERSION 2.8.3)
project(minho_team_tools CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation	
	sensor_msgs
	genmsg	
	minho_team_ros		
	cv_bridge
	image_transport
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

#add_message_files(
#		FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
	#generate_messages( DEPENDENCIES std_msgs sensor_msgs )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
	catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES minho_team
		CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
		DEPENDS sensor_msgs
	)


## DEPENDENCIES ##
######################################################################
set(PROJECT_DIRECTORY ${minho_team_tools_SOURCE_DIR})
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

set(username $ENV{USER})
set(home $ENV{HOME})
MESSAGE( STATUS "-----> Compiling MinhoTeam Packages for : " ${username})

#Linking steps for Qt Libraries
########################################
find_package(Qt5 COMPONENTS Core Widgets Gui PrintSupport REQUIRED)
add_definitions (${QT_DEFINITIONS})
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(qt_LIBRARIES Qt5::Widgets Qt5::Core Qt5::Gui Qt5::PrintSupport)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})
include_directories(${Qt5PrintSupport_INCLUDE_DIRS})
########################################

#Linking steps for OpenCV Libraries
########################################
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set (opencv_LIBRARIES ${OpenCV_LIBS})
########################################

include_directories($ENV{HOME}/Common)

#Linking steps for QCUSTOMPLOT Libraries
########################################
include_directories(${minho_team_tools_SOURCE_DIR}/libs)
##### libqcustomplot #####
##################
add_library(qcustomplot SHARED ${minho_team_tools_SOURCE_DIR}/libs/qcustomplot.cpp)
set_target_properties( qcustomplot
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${minho_team_tools_SOURCE_DIR}/libs
)
target_compile_features(qcustomplot PRIVATE cxx_range_for)
target_link_libraries(qcustomplot ${qt_LIBRARIES})
set (qcustomplot_LIBRARIES ${minho_team_tools_SOURCE_DIR}/libs/libqcustomplot.so)
########################################
######################################################################

## TELEOP ##
######################################################################
set(TELEOP_DIR ${PROJECT_DIRECTORY}/minho_teleop)
set(teleop_MOC_H ${TELEOP_DIR}/include/mainwindow.h)
qt5_wrap_cpp(teleop_CPP ${teleop_MOC_H})
set(teleop_SRC ${TELEOP_DIR}/src/main.cpp ${TELEOP_DIR}/src/mainwindow.cpp )
add_executable(teleop ${teleop_SRC} ${teleop_CPP}) 
target_include_directories(teleop PUBLIC
$<BUILD_INTERFACE:${TELEOP_DIR}/include>)
set_target_properties(teleop PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(teleop PRIVATE cxx_range_for)
target_link_libraries(teleop ${catkin_LIBRARIES} ${qt_LIBRARIES})
add_dependencies(teleop minho_team_ros_generate_messages_cpp)
######################################################################

## VISION CALIB ##
######################################################################
set(VISIONCALIB_DIR ${PROJECT_DIRECTORY}/vision_calib)
set(visioncalib_MOC_H ${VISIONCALIB_DIR}/include/mainwindow.h ${VISIONCALIB_DIR}/include/imagecalibrator.h)
qt5_wrap_cpp(visioncalib_CPP ${visioncalib_MOC_H})
set(visioncalib_SRC ${VISIONCALIB_DIR}/src/main.cpp ${VISIONCALIB_DIR}/src/mainwindow.cpp ${VISIONCALIB_DIR}/src/imagecalibrator.cpp) 
add_executable(vision_calib ${visioncalib_SRC} ${visioncalib_CPP}) 
target_include_directories(vision_calib PUBLIC
$<BUILD_INTERFACE:${VISIONCALIB_DIR}/include>)
set_target_properties(vision_calib PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(vision_calib PRIVATE cxx_range_for)
target_link_libraries(vision_calib ${catkin_LIBRARIES} ${qt_LIBRARIES} ${opencv_LIBRARIES} ${qcustomplot_LIBRARIES})
add_dependencies(vision_calib minho_team_ros_generate_messages_cpp qcustomplot)
######################################################################

## VISUALIZER ##
######################################################################
set(MTROS_DIR ${PROJECT_DIRECTORY}/../minho_team_ros)
include_directories(${MTROS_DIR}/coms_node/include)
set(VIS_DIR ${PROJECT_DIRECTORY}/visualizer)
set(vis_MOC_H ${VIS_DIR}/include/visualizer.h)
qt5_wrap_cpp(vis_CPP ${vis_MOC_H})
set(vis_SRC ${VIS_DIR}/src/main.cpp ${VIS_DIR}/src/visualizer.cpp ${MTROS_DIR}/coms_node/src/multicast.cpp)
add_executable(visualizer ${vis_SRC} ${vis_CPP}) 
target_include_directories(visualizer PUBLIC
$<BUILD_INTERFACE:${VIS_DIR}/include>)
set_target_properties(visualizer PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(visualizer PRIVATE cxx_range_for)
target_link_libraries(visualizer ${catkin_LIBRARIES} ${qt_LIBRARIES} ${opencv_LIBRARIES})
add_dependencies(visualizer minho_team_ros_generate_messages_cpp)
######################################################################

## MINI_BASESTATION CALIB ##
######################################################################
set(MINIBS_DIR ${PROJECT_DIRECTORY}/mini_basestation)
set(minibs_MOC_H ${MINIBS_DIR}/include/mainwindow.h)
qt5_wrap_cpp(minibs_CPP ${minibs_MOC_H})
set(minibs_SRC ${MINIBS_DIR}/src/main.cpp ${MINIBS_DIR}/src/mainwindow.cpp 
${MINIBS_DIR}/src/ball.cpp ${MINIBS_DIR}/src/customscene.cpp ${MINIBS_DIR}/src/desenharcampo.cpp 
${MINIBS_DIR}/src/multicastpp.cpp ${MINIBS_DIR}/src/robo.cpp ${MINIBS_DIR}/src/thpool.c 
${MINIBS_DIR}/src/velocityvector.cpp ${MINIBS_DIR}/config/rs.qrc) 
add_executable(mini_basestation ${minibs_SRC} ${minibs_CPP}) 
target_include_directories(mini_basestation PUBLIC
$<BUILD_INTERFACE:${MINIBS_DIR}/include>)
set_target_properties(mini_basestation PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(mini_basestation PRIVATE cxx_range_for)
target_link_libraries(mini_basestation ${catkin_LIBRARIES} ${qt_LIBRARIES})
add_dependencies(mini_basestation minho_team_ros_generate_messages_cpp)
######################################################################

## CONTROL ##
######################################################################
set(CCALIB_DIR ${PROJECT_DIRECTORY}/control_calib)
set(ccalib_MOC_H ${CCALIB_DIR}/include/mainwindow.h)
qt5_wrap_cpp(ccalib_CPP ${ccalib_MOC_H})
set(ccalib_SRC ${CCALIB_DIR}/src/main.cpp ${CCALIB_DIR}/src/mainwindow.cpp)
add_executable(control_calib ${ccalib_SRC} ${ccalib_CPP}) 
target_include_directories(control_calib PUBLIC
$<BUILD_INTERFACE:${CCALIB_DIR}/include>)
set_target_properties(control_calib PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(control_calib PRIVATE cxx_range_for)
target_link_libraries(control_calib ${catkin_LIBRARIES} ${qt_LIBRARIES})
add_dependencies(control_calib minho_team_ros_generate_messages_cpp)
######################################################################

## MAPGEN ##
######################################################################
set(MPGEN_SRCDIR ${PROJECT_DIRECTORY}/minhomapgen/src)
set(mpgen_MOC_H ${PROJECT_DIRECTORY}/minhomapgen/include/mainwindow.h)
qt5_wrap_cpp(mpgen_CPP ${mpgen_MOC_H})
set(mpgen_SRC ${MPGEN_SRCDIR}/main.cpp ${MPGEN_SRCDIR}/mainwindow.cpp)
add_executable(minhomapgen ${mpgen_SRC} ${mpgen_CPP}) 
target_include_directories(minhomapgen PUBLIC
$<BUILD_INTERFACE:${PROJECT_DIRECTORY}/minhomapgen/include>)

set_target_properties(minhomapgen PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(minhomapgen PRIVATE cxx_range_for)
target_link_libraries(minhomapgen ${qt_LIBRARIES} ${opencv_LIBRARIES})
######################################################################
