# Install script for directory: /home/cs4750/catkin_ws/src/foundations_hw1

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cs4750/catkin_ws/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  include("/home/cs4750/catkin_ws/build/foundations_hw1/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/foundations_hw1/srv" TYPE FILE FILES
    "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv"
    "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/foundations_hw1/cmake" TYPE FILE FILES "/home/cs4750/catkin_ws/build/foundations_hw1/catkin_generated/installspace/foundations_hw1-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cs4750/catkin_ws/devel/include/foundations_hw1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cs4750/catkin_ws/devel/share/roseus/ros/foundations_hw1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cs4750/catkin_ws/devel/share/common-lisp/ros/foundations_hw1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cs4750/catkin_ws/devel/share/gennodejs/ros/foundations_hw1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/cs4750/catkin_ws/devel/lib/python2.7/dist-packages/foundations_hw1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cs4750/catkin_ws/devel/lib/python2.7/dist-packages/foundations_hw1" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cs4750/catkin_ws/devel/lib/python2.7/dist-packages/foundations_hw1" FILES_MATCHING REGEX "/home/cs4750/catkin_ws/devel/lib/python2.7/dist-packages/foundations_hw1/.+/__init__.pyc?$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cs4750/catkin_ws/build/foundations_hw1/catkin_generated/installspace/foundations_hw1.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/foundations_hw1/cmake" TYPE FILE FILES "/home/cs4750/catkin_ws/build/foundations_hw1/catkin_generated/installspace/foundations_hw1-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/foundations_hw1/cmake" TYPE FILE FILES
    "/home/cs4750/catkin_ws/build/foundations_hw1/catkin_generated/installspace/foundations_hw1Config.cmake"
    "/home/cs4750/catkin_ws/build/foundations_hw1/catkin_generated/installspace/foundations_hw1Config-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/foundations_hw1" TYPE FILE FILES "/home/cs4750/catkin_ws/src/foundations_hw1/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/foundations_hw1" TYPE PROGRAM FILES
    "/home/cs4750/catkin_ws/src/foundations_hw1/bin/turtle_target"
    "/home/cs4750/catkin_ws/src/foundations_hw1/bin/tag_launcher"
    "/home/cs4750/catkin_ws/src/foundations_hw1/bin/turtle_tag"
    "/home/cs4750/catkin_ws/src/foundations_hw1/bin/turtle_tag_chaser"
    "/home/cs4750/catkin_ws/src/foundations_hw1/bin/pub_rate"
    "/home/cs4750/catkin_ws/src/foundations_hw1/bin/image_sensor"
    "/home/cs4750/catkin_ws/src/foundations_hw1/bin/reward_func"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/foundations_hw1" TYPE FILE FILES
    "/home/cs4750/catkin_ws/src/foundations_hw1/launch/p2c.launch"
    "/home/cs4750/catkin_ws/src/foundations_hw1/launch/p3b.launch"
    "/home/cs4750/catkin_ws/src/foundations_hw1/launch/p4b.launch"
    "/home/cs4750/catkin_ws/src/foundations_hw1/launch/p4c.launch"
    "/home/cs4750/catkin_ws/src/foundations_hw1/launch/p5c.launch"
    "/home/cs4750/catkin_ws/src/foundations_hw1/launch/default.launch"
    )
endif()

