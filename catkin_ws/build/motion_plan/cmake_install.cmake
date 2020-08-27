<<<<<<< HEAD
# Install script for directory: /home/kevin/autonav_sim/catkin_ws/src/motion_plan

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kevin/autonav_sim/catkin_ws/install")
=======
# Install script for directory: /home/akshat/Anveshak/sim/autonav_sim/catkin_ws/src/motion_plan

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
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
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kevin/autonav_sim/catkin_ws/build/motion_plan/catkin_generated/installspace/motion_plan.pc")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/motion_plan/catkin_generated/installspace/motion_plan.pc")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motion_plan/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/kevin/autonav_sim/catkin_ws/build/motion_plan/catkin_generated/installspace/motion_planConfig.cmake"
    "/home/kevin/autonav_sim/catkin_ws/build/motion_plan/catkin_generated/installspace/motion_planConfig-version.cmake"
=======
    "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/motion_plan/catkin_generated/installspace/motion_planConfig.cmake"
    "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/motion_plan/catkin_generated/installspace/motion_planConfig-version.cmake"
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motion_plan" TYPE FILE FILES "/home/kevin/autonav_sim/catkin_ws/src/motion_plan/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motion_plan" TYPE FILE FILES "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/src/motion_plan/package.xml")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
endif()

