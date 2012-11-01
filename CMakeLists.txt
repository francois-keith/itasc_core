cmake_minimum_required(VERSION 2.6.3)

project(itasc_core)

set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  set(ROS_BUILD_TYPE Release) #set type to Debug to get debug info compiled
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
endif()

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_use_package(rtt-marshalling)
orocos_component(iTaSC_Scene src/Scene.hpp src/Scene.cpp )
target_link_libraries(iTaSC_Scene eigen_toolkit)
orocos_install_headers( src/ConstraintController.hpp )
orocos_install_headers( src/ConstraintControllerInequality.hpp )
orocos_install_headers( src/VirtualKinematicChain.hpp )
orocos_install_headers( src/Solver.hpp )
orocos_install_headers( src/SubRobot.hpp )
#TODO SHOULD BE DELETED WHEN CHOLESKI MOVED TO KDL
orocos_install_headers( src/choleski_semidefinite.hpp src/eigen_toolkit.hpp)

orocos_typekit(eigen_toolkit src/eigen_toolkit.cpp)

#TODO DELETE THIS WHEN NEW KDL VERSION WITH CHOLESKI IS STABLE
orocos_library(choleski_semidefinite src/choleski_semidefinite.cpp)
orocos_generate_package()
