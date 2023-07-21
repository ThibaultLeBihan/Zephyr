# Install script for directory: /home/hydrosharks2/workspaceRos/src/plymouth_internship_2019

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/hydrosharks2/workspaceRos/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plymouth_internship_2019/msg" TYPE FILE FILES "/home/hydrosharks2/workspaceRos/src/plymouth_internship_2019/msg/KeyboardServoCommand.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plymouth_internship_2019/cmake" TYPE FILE FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/hydrosharks2/workspaceRos/devel/include/plymouth_internship_2019")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/hydrosharks2/workspaceRos/devel/share/roseus/ros/plymouth_internship_2019")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/hydrosharks2/workspaceRos/devel/share/common-lisp/ros/plymouth_internship_2019")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/hydrosharks2/workspaceRos/devel/share/gennodejs/ros/plymouth_internship_2019")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/hydrosharks2/workspaceRos/devel/lib/python2.7/dist-packages/plymouth_internship_2019")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/hydrosharks2/workspaceRos/devel/lib/python2.7/dist-packages/plymouth_internship_2019")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plymouth_internship_2019/cmake" TYPE FILE FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plymouth_internship_2019/cmake" TYPE FILE FILES
    "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019Config.cmake"
    "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plymouth_internship_2019" TYPE FILE FILES "/home/hydrosharks2/workspaceRos/src/plymouth_internship_2019/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plymouth_internship_2019/cmake" TYPE FILE FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plymouth_internship_2019/cmake" TYPE FILE FILES
    "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019Config.cmake"
    "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/plymouth_internship_2019Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plymouth_internship_2019" TYPE FILE FILES "/home/hydrosharks2/workspaceRos/src/plymouth_internship_2019/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/plymouth_internship_2019" TYPE PROGRAM FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/coordinator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/plymouth_internship_2019" TYPE PROGRAM FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/sailboat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/plymouth_internship_2019" TYPE PROGRAM FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/testSailboat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/plymouth_internship_2019" TYPE PROGRAM FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/operator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/plymouth_internship_2019" TYPE PROGRAM FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/fleetCoordinator")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/plymouth_internship_2019" TYPE PROGRAM FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/fleetSailboat")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/plymouth_internship_2019" TYPE PROGRAM FILES "/home/hydrosharks2/workspaceRos/build/plymouth_internship_2019/catkin_generated/installspace/imageProcessing")
endif()

