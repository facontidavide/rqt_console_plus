##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(rqt_console_plus)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -DUSE_ROSOUT2=1")

find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5Widgets)
find_package(Qt5Xml)

set( QT_LIBRARIES
    ${Qt5Core_QTMAIN_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
)

# Qt related stuff
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    rosbag
    rosbag_storage
    rosout2_msg  )

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 

catkin_package(
     #    INCLUDE_DIRS include
     #   LIBRARIES
        CATKIN_DEPENDS roscpp rosbag rosbag_storage rosout2_msg
    #     DEPENDS Boost PythonLibs
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt5(QtCore QtGui) # Add the appropriate components to the component list here


##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/rqt_console_plus/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(rqt_console_plus ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(rqt_console_plus ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS rqt_console_plus RUNTIME DESTINATION /usr/local/bin)


#########################  create a package ####################################

SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Davide Faconti")
SET(CPACK_PACKAGE_CONTACT "faconti@icarustechnology.com")

INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGING_INSTALL_PREFIX "/usr/local/")
SET(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)

SET(CPACK_DEBIAN_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "rqt_console_plus")
SET(CPACK_PACKAGE_VENDOR "Icarus Technology")

SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "3")
SET(CPACK_PACKAGE_VERSION_PATCH "0")
SET(CPACK_PACKAGE_VERSION "0.3.0")

SET(CPACK_PACKAGE_NAME "rqt_console_plus${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

SET(CPACK_DEBIAN_PACKAGE_DEPENDS "qt5-default")


INCLUDE(CPack)

