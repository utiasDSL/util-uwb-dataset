# Install script for directory: /home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cf_msgs/msg" TYPE FILE FILES
    "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg"
    "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg"
    "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg"
    "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg"
    "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg"
    "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cf_msgs/cmake" TYPE FILE FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/build/cf_msgs/catkin_generated/installspace/cf_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/devel/include/cf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/devel/share/roseus/ros/cf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/devel/share/common-lisp/ros/cf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/devel/share/gennodejs/ros/cf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/devel/lib/python2.7/dist-packages/cf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/devel/lib/python2.7/dist-packages/cf_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/build/cf_msgs/catkin_generated/installspace/cf_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cf_msgs/cmake" TYPE FILE FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/build/cf_msgs/catkin_generated/installspace/cf_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cf_msgs/cmake" TYPE FILE FILES
    "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/build/cf_msgs/catkin_generated/installspace/cf_msgsConfig.cmake"
    "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/build/cf_msgs/catkin_generated/installspace/cf_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cf_msgs" TYPE FILE FILES "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/package.xml")
endif()

