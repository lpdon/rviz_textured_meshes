cmake_minimum_required(VERSION 2.8.3)
project(rviz_textured_meshes)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++11)
## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib std_msgs sensor_msgs shape_msgs rviz pluginlib class_loader cv_bridge message_generation geometry_msgs sensor_msgs cmake_modules eigen_conversions roslint rviz)

find_package(Eigen REQUIRED)
find_package(OpenGL REQUIRED)

find_package(Boost REQUIRED filesystem)

roslint_cpp()
roslint_python()

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
)

include_directories(
	src
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

if(rviz_QT_VERSION VERSION_LESS "5")
qt4_wrap_cpp(MOC_FILES
  include/rviz_textured_meshes/mesh_display_custom.h
  )
else()
qt5_wrap_cpp(MOC_FILES
  include/rviz_textured_meshes/mesh_display_custom.h
  )
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(TEXTURED_MESH_LIB rviz_textured_meshes)

add_library(${TEXTURED_MESH_LIB} src/mesh_display_custom.cpp ${MOC_FILES})
target_link_libraries(${TEXTURED_MESH_LIB} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${TEXTURED_MESH_LIB} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${TEXTURED_MESH_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
  mesh_display_custom_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
