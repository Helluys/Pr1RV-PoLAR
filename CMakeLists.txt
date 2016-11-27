CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

PROJECT(LiveAR)

set(PROJECT_VERSION 1.0)
set(CMAKE_COLOR_MAKEFILE ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# This allows to remove the "QT5_WRAP_CPP" macros
set(CMAKE_AUTOMOC ON)

if(APPLE)
  set(CMAKE_MACOSX_RPATH OFF) #disable MAC OS environment variable
endif(APPLE)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

if(CMAKE_COMPILER_IS_GNUCXX)
    include(GNUInstallDirs)
endif(CMAKE_COMPILER_IS_GNUCXX)


set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin
  CACHE PATH "Single output directory for building all executables.")
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib
  CACHE PATH "Single output directory for PoLAR library.")

if(NOT WIN32)
        set(CMAKE_CXX_FLAGS "-pipe -Wall -Wextra -std=c++11 -pedantic"
                CACHE STRING "Custom flags used by the compiler during all build types." FORCE)
        set(CMAKE_CXX_FLAGS_DEBUG "-g3 -Wall -Wextra -ggdb -Wno-reorder -std=c++11 -Woverloaded-virtual -pedantic"
                CACHE STRING "Custom flags used by the compiler during debug builds." FORCE)
endif(NOT WIN32)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release
    CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_INCLUDE_PATH
  ${CMAKE_INCLUDE_PATH}
  /usr/local/include
  /usr/include
  )

set(CMAKE_LIBRARY_PATH
  ${CMAKE_LIBRARY_PATH}
  /usr/lib
  /usr/local/lib
  )

if(APPLE)
  set(CMAKE_INCLUDE_PATH
    ${CMAKE_INCLUDE_PATH}
    /usr/local/include/
    /usr/local/Cellar/open-scene-graph/3.2.0/include/
    )

  set(CMAKE_LIBRARY_PATH
    ${CMAKE_LIBRARY_PATH}
    /usr/local/Cellar/open-scene-graph/3.2.0/lib/
    /usr/local/lib/
    )
endif(APPLE)

# Adding some macros
if(WIN32)
    add_definitions(-DWIN32)
endif(WIN32)
if(APPLE)
    add_definitions(-DAPPLE)
endif(APPLE)

# QT5 modules
option(USE_QT_PACKAGES "Use packaged version of Qt (Linux/Mac)" ON)

if(WIN32 OR NOT USE_QT_PACKAGES)
  set(QT_CUSTOM_DIR "C:\\Qt\\Qt5.5.0-x86\\5.5\\msvc2013" CACHE STRING "If not using Qt in package version, set Qt install dir")
endif()

if(APPLE OR NOT USE_QT_PACKAGES)
  set(QT_CUSTOM_DIR "/usr/local/Cellar/qt5/5.5.1_2/" CACHE STRING "If not using Qt in package version, set Qt install dir")
endif()

set(CMAKE_PREFIX_PATH ${QT_CUSTOM_DIR})

find_package(Qt5Core REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5Multimedia REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Svg REQUIRED)

get_target_property(QtCore_location Qt5::Core LOCATION)
get_target_property(QtOpenGL_location Qt5::OpenGL LOCATION)
get_target_property(QtXml_location Qt5::Xml LOCATION)
get_target_property(QtGui_location Qt5::Gui LOCATION)
get_target_property(QtMultimedia_location Qt5::Multimedia LOCATION)
get_target_property(QtWidgets_location Qt5::Widgets LOCATION)
get_target_property(QtSvg_location Qt5::Svg LOCATION)

add_definitions(${Qt5Core_DEFINITIONS})
add_definitions(${Qt5OpenGL_DEFINITIONS})
add_definitions(${Qt5Xml_DEFINITIONS})
add_definitions(${Qt5Gui_DEFINITIONS})
add_definitions(${Qt5Multimedia_DEFINITIONS})
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5Svg_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Core_EXECUTABLE_COMPILE_FLAGS}")

set(QT_INCLUDES
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Xml_INCLUDE_DIRS}
  ${Qt5Multimedia_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Svg_INCLUDE_DIRS}
)

set(QT_LIBRARIES
  ${Qt5Core_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  ${Qt5Xml_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  ${Qt5Multimedia_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Svg_LIBRARIES}
)

# OpenSceneGraph modules
option(USE_OSG_PACKAGES "Use packaged version of OpenSceneGraph (Linux/Mac)" ON)

if(WIN32 OR NOT USE_OSG_PACKAGES)
  set(OSG_CUSTOM_DIR "C:\\OpenSceneGraph-3.4.0\\build" CACHE STRING "When not using OSG package version, set OSG install dir")
  set(OSG_CUSTOM_INCLUDE_DIRS
         "C:\\OpenSceneGraph-3.4.0\\include"
         "C:\\OpenSceneGraph-3.4.0\\build\\include"
                 CACHE STRING "When not using OSG package version, set OSG include dir"
  )
endif()

set(CMAKE_PREFIX_PATH ${OSG_CUSTOM_DIR})
include_directories(${OSG_CUSTOM_INCLUDE_DIRS})

# Find custom libraries thanks to their names and put them (their location path) in the corresponding variable
find_package(OpenSceneGraph COMPONENTS
  osgViewer
  osgGA
  osgDB
  osgText
  osgUtil
  osgManipulator
  osgShadow
  osgAnimation
  OpenThreads
)

if(NOT OPENSCENEGRAPH_FOUND)
  find_library(OSG_LIBRARY osg)
  find_library(OSGVIEWER_LIBRARY osgViewer)
  find_library(OSGGA_LIBRARY osgGA)
  find_library(OSGDB_LIBRARY osgDB)
  find_library(OSGTEXT_LIBRARY osgText)
  find_library(OSGUTIL_LIBRARY osgUtil)
  find_library(OSGMANIPULATOR_LIBRARY osgManipulator)
  find_library(OSGSHADOW_LIBRARY osgShadow)
  find_library(OSGANIMATION_LIBRARY osgAnimation)
  find_library(OPENTHREADS_LIBRARY OpenThreads)
endif(NOT OPENSCENEGRAPH_FOUND)

set(OPENSCENEGRAPH_LIBRARIES
  ${OSG_LIBRARY}
  ${OSGVIEWER_LIBRARY}
  ${OSGGA_LIBRARY}
  ${OSGDB_LIBRARY}
  ${OSGTEXT_LIBRARY}
  ${OSGUTIL_LIBRARY}
  ${OSGMANIPULATOR_LIBRARY}
  ${OSGSHADOW_LIBRARY}
  ${OSGANIMATION_LIBRARY}
  ${OPENTHREADS_LIBRARY}
)

if(APPLE)
  set(OSG_INCLUDES
    ${OSG_INCLUDE_DIR}
    ${OSGVIEWER_INCLUDE_DIR}
    ${OSGGA_INCLUDE_DIR}
    ${OSGDB_INCLUDE_DIR}
    ${OSGTEXT_INCLUDE_DIR}
    ${OSGUTIL_INCLUDE_DIR}
    ${OSGMANIPULATOR_INCLUDE_DIR}
    ${OSGSHADOW_INCLUDE_DIR}
    ${OSGANIMATION_INCLUDE_DIR}
    ${OPENTHREADS_INCLUDE_DIR}
  )

  include_directories(
    ${OSG_INCLUDES}
  )
endif()

# Add the FindPoLAR.cmake
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules
)


# PoLAR
find_package(PoLAR REQUIRED)
if(NOT PoLAR_FOUND)
  message(WARNING "PoLAR not found")
else()
  include_directories(${PoLAR_INCLUDE_DIR})
  link_directories(${PoLAR_LIBRARY_DIR})
endif()

# OpenCV
find_package( OpenCV REQUIRED )

set(LINK_LIBRARIES
  ${LINK_LIBRARIES}
  ${PoLAR_LIBRARIES}
  ${QT_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${OPENSCENEGRAPH_LIBRARIES}
  ${OpenCV_LIBS}
)


set(SRCS
  src/LiveARApp.h
  src/LiveARApp.cpp
  src/LiveARViewer.h
  src/LiveARViewer.cpp
  src/LiveARTracker.h
  src/LiveARTracker.cpp
)


# executable
ADD_EXECUTABLE(LiveAR ${SRCS} src/main.cpp)
target_include_directories(LiveAR PUBLIC
  ${QT_INCLUDES}
  ${OSG_INCLUDES}
  ${PoLAR_INCLUDE_DIR}
)
target_link_libraries(LiveAR ${LINK_LIBRARIES})



