<<<<<<< HEAD
# Install script for directory: /home/kevin/autonav_sim/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kevin/autonav_sim/catkin_ws/install")
=======
# Install script for directory: /home/akshat/Anveshak/sim/autonav_sim/catkin_ws/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/kevin/autonav_sim/catkin_ws/install/_setup_util.py")
=======
   "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/_setup_util.py")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/kevin/autonav_sim/catkin_ws/install" TYPE PROGRAM FILES "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/_setup_util.py")
=======
file(INSTALL DESTINATION "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install" TYPE PROGRAM FILES "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/_setup_util.py")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/kevin/autonav_sim/catkin_ws/install/env.sh")
=======
   "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/env.sh")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/kevin/autonav_sim/catkin_ws/install" TYPE PROGRAM FILES "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/env.sh")
=======
file(INSTALL DESTINATION "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install" TYPE PROGRAM FILES "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/env.sh")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/kevin/autonav_sim/catkin_ws/install/setup.bash;/home/kevin/autonav_sim/catkin_ws/install/local_setup.bash")
=======
   "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/setup.bash;/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/local_setup.bash")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/kevin/autonav_sim/catkin_ws/install" TYPE FILE FILES
    "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/setup.bash"
    "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/local_setup.bash"
=======
file(INSTALL DESTINATION "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install" TYPE FILE FILES
    "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/setup.bash"
    "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/local_setup.bash"
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/kevin/autonav_sim/catkin_ws/install/setup.sh;/home/kevin/autonav_sim/catkin_ws/install/local_setup.sh")
=======
   "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/setup.sh;/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/local_setup.sh")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/kevin/autonav_sim/catkin_ws/install" TYPE FILE FILES
    "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/setup.sh"
    "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/local_setup.sh"
=======
file(INSTALL DESTINATION "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install" TYPE FILE FILES
    "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/setup.sh"
    "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/local_setup.sh"
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/kevin/autonav_sim/catkin_ws/install/setup.zsh;/home/kevin/autonav_sim/catkin_ws/install/local_setup.zsh")
=======
   "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/setup.zsh;/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/local_setup.zsh")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/kevin/autonav_sim/catkin_ws/install" TYPE FILE FILES
    "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/local_setup.zsh"
=======
file(INSTALL DESTINATION "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install" TYPE FILE FILES
    "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/local_setup.zsh"
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/kevin/autonav_sim/catkin_ws/install/.rosinstall")
=======
   "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install/.rosinstall")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/kevin/autonav_sim/catkin_ws/install" TYPE FILE FILES "/home/kevin/autonav_sim/catkin_ws/build/catkin_generated/installspace/.rosinstall")
=======
file(INSTALL DESTINATION "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/install" TYPE FILE FILES "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/catkin_generated/installspace/.rosinstall")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
<<<<<<< HEAD
  include("/home/kevin/autonav_sim/catkin_ws/build/gtest/cmake_install.cmake")
  include("/home/kevin/autonav_sim/catkin_ws/build/motion_plan/cmake_install.cmake")
=======
  include("/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/gtest/cmake_install.cmake")
  include("/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/motion_plan/cmake_install.cmake")
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/kevin/autonav_sim/catkin_ws/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/akshat/Anveshak/sim/autonav_sim/catkin_ws/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> b6c1e11b6156308465211a69bfcc0cfb0f45f8f5
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
