# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xmlrpc_wrapper: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igazebo_msgs:/opt/ros/melodic/share/gazebo_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xmlrpc_wrapper_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv" NAME_WE)
add_custom_target(_xmlrpc_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xmlrpc_wrapper" "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv" "gazebo_msgs/LinkState:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv" NAME_WE)
add_custom_target(_xmlrpc_wrapper_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xmlrpc_wrapper" "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv" "gazebo_msgs/LinkState:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xmlrpc_wrapper
)
_generate_srv_cpp(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xmlrpc_wrapper
)

### Generating Module File
_generate_module_cpp(xmlrpc_wrapper
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xmlrpc_wrapper
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xmlrpc_wrapper_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xmlrpc_wrapper_generate_messages xmlrpc_wrapper_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_cpp _xmlrpc_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_cpp _xmlrpc_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xmlrpc_wrapper_gencpp)
add_dependencies(xmlrpc_wrapper_gencpp xmlrpc_wrapper_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xmlrpc_wrapper_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xmlrpc_wrapper
)
_generate_srv_eus(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xmlrpc_wrapper
)

### Generating Module File
_generate_module_eus(xmlrpc_wrapper
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xmlrpc_wrapper
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xmlrpc_wrapper_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xmlrpc_wrapper_generate_messages xmlrpc_wrapper_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_eus _xmlrpc_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_eus _xmlrpc_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xmlrpc_wrapper_geneus)
add_dependencies(xmlrpc_wrapper_geneus xmlrpc_wrapper_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xmlrpc_wrapper_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xmlrpc_wrapper
)
_generate_srv_lisp(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xmlrpc_wrapper
)

### Generating Module File
_generate_module_lisp(xmlrpc_wrapper
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xmlrpc_wrapper
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xmlrpc_wrapper_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xmlrpc_wrapper_generate_messages xmlrpc_wrapper_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_lisp _xmlrpc_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_lisp _xmlrpc_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xmlrpc_wrapper_genlisp)
add_dependencies(xmlrpc_wrapper_genlisp xmlrpc_wrapper_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xmlrpc_wrapper_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xmlrpc_wrapper
)
_generate_srv_nodejs(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xmlrpc_wrapper
)

### Generating Module File
_generate_module_nodejs(xmlrpc_wrapper
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xmlrpc_wrapper
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xmlrpc_wrapper_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xmlrpc_wrapper_generate_messages xmlrpc_wrapper_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_nodejs _xmlrpc_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_nodejs _xmlrpc_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xmlrpc_wrapper_gennodejs)
add_dependencies(xmlrpc_wrapper_gennodejs xmlrpc_wrapper_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xmlrpc_wrapper_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xmlrpc_wrapper
)
_generate_srv_py(xmlrpc_wrapper
  "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/gazebo_msgs/cmake/../msg/LinkState.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xmlrpc_wrapper
)

### Generating Module File
_generate_module_py(xmlrpc_wrapper
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xmlrpc_wrapper
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xmlrpc_wrapper_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xmlrpc_wrapper_generate_messages xmlrpc_wrapper_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetReelPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_py _xmlrpc_wrapper_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/xmlrpc_wrapper/srv/GetCharucoPose.srv" NAME_WE)
add_dependencies(xmlrpc_wrapper_generate_messages_py _xmlrpc_wrapper_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xmlrpc_wrapper_genpy)
add_dependencies(xmlrpc_wrapper_genpy xmlrpc_wrapper_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xmlrpc_wrapper_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xmlrpc_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xmlrpc_wrapper
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xmlrpc_wrapper_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET gazebo_msgs_generate_messages_cpp)
  add_dependencies(xmlrpc_wrapper_generate_messages_cpp gazebo_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xmlrpc_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xmlrpc_wrapper
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xmlrpc_wrapper_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET gazebo_msgs_generate_messages_eus)
  add_dependencies(xmlrpc_wrapper_generate_messages_eus gazebo_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xmlrpc_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xmlrpc_wrapper
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xmlrpc_wrapper_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET gazebo_msgs_generate_messages_lisp)
  add_dependencies(xmlrpc_wrapper_generate_messages_lisp gazebo_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xmlrpc_wrapper)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xmlrpc_wrapper
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xmlrpc_wrapper_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET gazebo_msgs_generate_messages_nodejs)
  add_dependencies(xmlrpc_wrapper_generate_messages_nodejs gazebo_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xmlrpc_wrapper)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xmlrpc_wrapper\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xmlrpc_wrapper
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xmlrpc_wrapper_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET gazebo_msgs_generate_messages_py)
  add_dependencies(xmlrpc_wrapper_generate_messages_py gazebo_msgs_generate_messages_py)
endif()
