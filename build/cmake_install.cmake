# Install script for directory: /home/brad/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/brad/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
   "/home/brad/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/brad/catkin_ws/install" TYPE PROGRAM FILES "/home/brad/catkin_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/brad/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/brad/catkin_ws/install" TYPE PROGRAM FILES "/home/brad/catkin_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/brad/catkin_ws/install/setup.bash;/home/brad/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/brad/catkin_ws/install" TYPE FILE FILES
    "/home/brad/catkin_ws/build/catkin_generated/installspace/setup.bash"
    "/home/brad/catkin_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/brad/catkin_ws/install/setup.sh;/home/brad/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/brad/catkin_ws/install" TYPE FILE FILES
    "/home/brad/catkin_ws/build/catkin_generated/installspace/setup.sh"
    "/home/brad/catkin_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/brad/catkin_ws/install/setup.zsh;/home/brad/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/brad/catkin_ws/install" TYPE FILE FILES
    "/home/brad/catkin_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/brad/catkin_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/brad/catkin_ws/install/setup.fish;/home/brad/catkin_ws/install/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/brad/catkin_ws/install" TYPE FILE FILES
    "/home/brad/catkin_ws/build/catkin_generated/installspace/setup.fish"
    "/home/brad/catkin_ws/build/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/brad/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/brad/catkin_ws/install" TYPE FILE FILES "/home/brad/catkin_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/brad/catkin_ws/build/gtest/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/bunker/bunker_description/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/hunter/hunter2_control/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/hunter_se/hunter_se_control/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/hunter_se/hunter_se_gazebo/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/limo/limo_description/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/limo_s2/limo_s2_description/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/ranger_mini/ranger_mini_control/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/ranger_mini/ranger_mini_v2_gazebo/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/ranger_mini/ranger_mini_v3_gazebo/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/ranger_mini/four_wheel_steering_msgs/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/scout_ros/scout_msgs/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_sdk/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/scout_ros/scout_bringup/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/scout/scout_description/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/bunker/bunker_gazebo_sim/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/limo/limo_gazebo_sim/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/limo_s2/limo_s2_gazebo_sim/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/scout_ros/scout_base/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/scout/scout_gazebo_sim/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/hunter/hunter2_gazebo/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/scout_mini_nav/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/tracer/tracer_gazebo_sim/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/tracer_mini/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/autokit/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/hunter/hunter2_base/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/hunter_se/hunter_se_description/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/limo_s2/limo_s2_arm/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/ranger_mini/ranger_mini_v2/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/hunter/steer_bot_hardware_gazebo/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/scout/scout_control/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/vicon_bridge/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/scout_mini_optimized/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/hunter/steer_drive_controller/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/tracer/tracer_description/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/ranger_mini/urdf_geometry_parser/cmake_install.cmake")
  include("/home/brad/catkin_ws/build/ugv_gazebo_sim/ranger_mini/four_wheel_steering_controller/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/brad/catkin_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
