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
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(username $ENV{USER})
set(home $ENV{HOME})
MESSAGE( STATUS "-----> Compiling MinhoTeam Packages for : " ${username})


if(EXISTS "/opt/Qt" AND IS_DIRECTORY "/opt/Qt")
set(CURRQTPREFIX /opt/Qt)
endif()
if(EXISTS "/opt/Qt/5.7/gcc_64" AND IS_DIRECTORY "/opt/Qt/5.7/gcc_64")
set(CURRQTPREFIX /opt/Qt/5.7/gcc_64)
endif()
if(EXISTS "${home}/Qt/5.7/gcc_64" AND IS_DIRECTORY "${home}/Qt/5.7/gcc_64")
set(CURRQTPREFIX ${home}/Qt/5.7/gcc_64)
endif()

set(CMAKE_PREFIX_PATH ${CURRQTPREFIX})
find_package(Qt5 COMPONENTS Core SerialPort Network Widgets REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(QT_DIR ${CURRQTPREFIX}/include)
include_directories(${QT_DIR} ${QT_DIR}/QtCore ${QT_DIR}/QtConcurrent ${QT_DIR}/QtDesigner ${QT_DIR}/QtDesignerComponents
${QT_DIR}/QtGui ${QT_DIR}/QtNetwork ${QT_DIR}/QtSerialPort ${QT_DIR}/QtWebSockets ${QT_DIR}/QtWidgets ${QT_DIR}/QtXml)
set(qt_LIBRARIES libQt5SerialPort.so libQt5Core.so libQt5Widgets.so libQt5Network.so libQt5Gui.so libQt5Qml.so libQt5Quick.so)
link_directories(${CURRQTPREFIX}/lib)
add_definitions(-DQT_THREAD_SUPPORT)
set(CMAKE_BUILD_TYPE Debug)
######################################################################

#Linking steps for OpenCV Libraries
########################################
include_directories(/usr/local/include/opencv /usr/local/include/opencv2)
set (opencv_LIBRARIES libopencv_core.so libopencv_highgui.so libopencv_imgproc.so)
link_directories(/usr/local/lib)
link_directories(/usr/lib/x86_64-linux-gnu)
########################################
######################################################################

## TELEOP ##
######################################################################
set(TELEOP_DIR ${PROJECT_DIRECTORY}/minho_teleop)
set(teleop_MOC_H ${TELEOP_DIR}/include/mainwindow.h)
qt5_wrap_cpp(teleop_CPP ${teleop_MOC_H})
set(teleop_SRC ${TELEOP_DIR}/src/main.cpp ${TELEOP_DIR}/src/mainwindow.cpp )
add_executable(minho_teleop ${teleop_SRC} ${teleop_CPP}) 
target_include_directories(minho_teleop PUBLIC
$<BUILD_INTERFACE:${TELEOP_DIR}/include>)
set_target_properties(minho_teleop PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(minho_teleop PRIVATE cxx_range_for)
target_link_libraries(minho_teleop ${catkin_LIBRARIES} ${qt_LIBRARIES})
add_dependencies(minho_teleop minho_team_ros_generate_messages_cpp)
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
target_link_libraries(vision_calib ${catkin_LIBRARIES} ${qt_LIBRARIES} ${opencv_LIBRARIES})
add_dependencies(vision_calib minho_team_ros_generate_messages_cpp)
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
