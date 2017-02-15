cmake_minimum_required(VERSION 2.8.3)
project(minhomapgen CXX)

set(PROJECT_DIRECTORY ${minhomapgen_SOURCE_DIR})

######################################################################
#Linking steps for Qt Libraries
########################################
find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)
add_definitions (${QT_DEFINITIONS})
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(qt_LIBRARIES Qt5::Widgets Qt5::Core Qt5::Gui)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})
########################################

#Linking steps for OpenCV Libraries
########################################
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set (opencv_LIBRARIES ${OpenCV_LIBS})
########################################
######################################################################

set(MPGEN_SRCDIR ${PROJECT_DIRECTORY}/src)
set(mpgen_MOC_H ${PROJECT_DIRECTORY}/include/mainwindow.h)
qt5_wrap_cpp(mpgen_CPP ${mpgen_MOC_H})
set(mpgen_SRC ${MPGEN_SRCDIR}/main.cpp ${MPGEN_SRCDIR}/mainwindow.cpp)
add_executable(minhomapgen ${mpgen_SRC} ${mpgen_CPP}) 
target_include_directories(minhomapgen PUBLIC
$<BUILD_INTERFACE:${PROJECT_DIRECTORY}/include>)

set_target_properties(minhomapgen PROPERTIES COMPILE_FLAGS "-fPIC")
target_compile_features(minhomapgen PRIVATE cxx_range_for)
target_link_libraries(minhomapgen ${qt_LIBRARIES} ${opencv_LIBRARIES})
######################################################################
