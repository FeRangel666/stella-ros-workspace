cmake_minimum_required(VERSION 3.0.2)
project(SubirEscaleras)

find_package(catkin REQUIRED COMPONENTS
  op3_walking_module_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(GLOB PYTHON_SCRIPTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} scripts/*.py)
foreach(_script ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${_script}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()

