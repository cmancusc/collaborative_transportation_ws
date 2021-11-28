# Install script for directory: /home/federico/benzi_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/federico/benzi_ws/src/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/home/federico/benzi_ws/src/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/federico/benzi_ws/src/build/catkin_generated/installspace/setup.bash"
    "/home/federico/benzi_ws/src/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/federico/benzi_ws/src/build/catkin_generated/installspace/setup.sh"
    "/home/federico/benzi_ws/src/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/home/federico/benzi_ws/src/build/catkin_generated/installspace/setup.zsh"
    "/home/federico/benzi_ws/src/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/home/federico/benzi_ws/src/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/federico/benzi_ws/src/build/gtest/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/amr-ros-config-master/gazebo/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/amr-ros-config-master/launch/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/catkin_simple/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/mir_robot/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/mir_msgs/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/sdc21x0/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/universal_robots/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/Universal_Robots_ROS_Driver/ur_dashboard_msgs/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/kdl_parser/kdl_parser_py/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/mir_description/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/mir_gazebo/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/mir_navigation/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/rossini_task_db-master/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/arbiter/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/shared_autonomy/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/Universal_Robots_ROS_Driver/controller_stopper/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/mir_actions/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/ati_mini_45_ros/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/fdo6/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/simple_navigation_goals/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/teleop_twist_joy/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/mir_driver/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mir_robot/mir_dwb_critics/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/Universal_Robots_ROS_Driver/ur_calibration/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur_description/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur_gazebo/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/kdl_parser/kdl_parser/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/amr-ros-config-master/description/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/tanks_iso/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/mobile_robot/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/null_damping/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/redundancy_solver/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur10_moveit_config/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur10e_moveit_config/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/ur10e_sim/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur16e_moveit_config/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur3_moveit_config/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur3e_moveit_config/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur5_moveit_config/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur5e_moveit_config/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/Universal_Robots_ROS_Driver/ur_controllers/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/universal_robot/ur_kinematics/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/Universal_Robots_ROS_Driver/ur_robot_driver/cmake_install.cmake")
  include("/home/federico/benzi_ws/src/build/xmlrpc_wrapper/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/federico/benzi_ws/src/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
