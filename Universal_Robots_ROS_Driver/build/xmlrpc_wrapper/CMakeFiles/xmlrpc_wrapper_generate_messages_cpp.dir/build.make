# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/federico/benzi_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/federico/benzi_ws/src/build

# Utility rule file for xmlrpc_wrapper_generate_messages_cpp.

# Include the progress variables for this target.
include xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/progress.make

xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp: devel/include/xmlrpc_wrapper/GetReelPose.h
xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp: devel/include/xmlrpc_wrapper/GetCharucoPose.h


devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/xmlrpc_wrapper/GetReelPose.h: ../xmlrpc_wrapper/srv/GetReelPose.srv
devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/share/gazebo_msgs/msg/LinkState.msg
devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Twist.msg
devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Vector3.msg
devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Pose.msg
devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Quaternion.msg
devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Point.msg
devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/share/gencpp/msg.h.template
devel/include/xmlrpc_wrapper/GetReelPose.h: /opt/ros/melodic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/federico/benzi_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from xmlrpc_wrapper/GetReelPose.srv"
	cd /home/federico/benzi_ws/src/xmlrpc_wrapper && /home/federico/benzi_ws/src/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igazebo_msgs:/opt/ros/melodic/share/gazebo_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -p xmlrpc_wrapper -o /home/federico/benzi_ws/src/build/devel/include/xmlrpc_wrapper -e /opt/ros/melodic/share/gencpp/cmake/..

devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/xmlrpc_wrapper/GetCharucoPose.h: ../xmlrpc_wrapper/srv/GetCharucoPose.srv
devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/share/gazebo_msgs/msg/LinkState.msg
devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Twist.msg
devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Vector3.msg
devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Pose.msg
devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Quaternion.msg
devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/share/geometry_msgs/msg/Point.msg
devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/share/gencpp/msg.h.template
devel/include/xmlrpc_wrapper/GetCharucoPose.h: /opt/ros/melodic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/federico/benzi_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from xmlrpc_wrapper/GetCharucoPose.srv"
	cd /home/federico/benzi_ws/src/xmlrpc_wrapper && /home/federico/benzi_ws/src/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Igazebo_msgs:/opt/ros/melodic/share/gazebo_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -p xmlrpc_wrapper -o /home/federico/benzi_ws/src/build/devel/include/xmlrpc_wrapper -e /opt/ros/melodic/share/gencpp/cmake/..

xmlrpc_wrapper_generate_messages_cpp: xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp
xmlrpc_wrapper_generate_messages_cpp: devel/include/xmlrpc_wrapper/GetReelPose.h
xmlrpc_wrapper_generate_messages_cpp: devel/include/xmlrpc_wrapper/GetCharucoPose.h
xmlrpc_wrapper_generate_messages_cpp: xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/build.make

.PHONY : xmlrpc_wrapper_generate_messages_cpp

# Rule to build all files generated by this target.
xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/build: xmlrpc_wrapper_generate_messages_cpp

.PHONY : xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/build

xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/clean:
	cd /home/federico/benzi_ws/src/build/xmlrpc_wrapper && $(CMAKE_COMMAND) -P CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/clean

xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/depend:
	cd /home/federico/benzi_ws/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/federico/benzi_ws/src /home/federico/benzi_ws/src/xmlrpc_wrapper /home/federico/benzi_ws/src/build /home/federico/benzi_ws/src/build/xmlrpc_wrapper /home/federico/benzi_ws/src/build/xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : xmlrpc_wrapper/CMakeFiles/xmlrpc_wrapper_generate_messages_cpp.dir/depend
