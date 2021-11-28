# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arbiter: 3 messages, 3 services")

set(MSG_I_FLAGS "-Iarbiter:/home/federico/benzi_ws/src/arbiter/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Irossini_task_db:/home/federico/benzi_ws/src/rossini_task_db-master/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arbiter_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv" NAME_WE)
add_custom_target(_arbiter_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arbiter" "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv" "arbiter/mex:arbiter/mexArray"
)

get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg" NAME_WE)
add_custom_target(_arbiter_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arbiter" "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg" "rossini_task_db/Command:rossini_task_db/Interaction"
)

get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/com.srv" NAME_WE)
add_custom_target(_arbiter_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arbiter" "/home/federico/benzi_ws/src/arbiter/srv/com.srv" ""
)

get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mex.msg" NAME_WE)
add_custom_target(_arbiter_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arbiter" "/home/federico/benzi_ws/src/arbiter/msg/mex.msg" ""
)

get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg" NAME_WE)
add_custom_target(_arbiter_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arbiter" "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg" "arbiter/mex"
)

get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv" NAME_WE)
add_custom_target(_arbiter_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arbiter" "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv" "rossini_task_db/Command:arbiter/InteractionArray:rossini_task_db/Interaction"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter
)
_generate_msg_cpp(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter
)
_generate_msg_cpp(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter
)

### Generating Services
_generate_srv_cpp(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/com.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter
)
_generate_srv_cpp(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg;/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter
)
_generate_srv_cpp(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter
)

### Generating Module File
_generate_module_cpp(arbiter
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arbiter_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arbiter_generate_messages arbiter_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_cpp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_cpp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/com.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_cpp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mex.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_cpp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_cpp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_cpp _arbiter_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arbiter_gencpp)
add_dependencies(arbiter_gencpp arbiter_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arbiter_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter
)
_generate_msg_eus(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter
)
_generate_msg_eus(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter
)

### Generating Services
_generate_srv_eus(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/com.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter
)
_generate_srv_eus(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg;/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter
)
_generate_srv_eus(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter
)

### Generating Module File
_generate_module_eus(arbiter
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arbiter_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arbiter_generate_messages arbiter_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_eus _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_eus _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/com.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_eus _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mex.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_eus _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_eus _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_eus _arbiter_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arbiter_geneus)
add_dependencies(arbiter_geneus arbiter_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arbiter_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter
)
_generate_msg_lisp(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter
)
_generate_msg_lisp(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter
)

### Generating Services
_generate_srv_lisp(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/com.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter
)
_generate_srv_lisp(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg;/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter
)
_generate_srv_lisp(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter
)

### Generating Module File
_generate_module_lisp(arbiter
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arbiter_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arbiter_generate_messages arbiter_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_lisp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_lisp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/com.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_lisp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mex.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_lisp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_lisp _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_lisp _arbiter_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arbiter_genlisp)
add_dependencies(arbiter_genlisp arbiter_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arbiter_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter
)
_generate_msg_nodejs(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter
)
_generate_msg_nodejs(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter
)

### Generating Services
_generate_srv_nodejs(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/com.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter
)
_generate_srv_nodejs(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg;/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter
)
_generate_srv_nodejs(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter
)

### Generating Module File
_generate_module_nodejs(arbiter
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arbiter_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arbiter_generate_messages arbiter_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_nodejs _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_nodejs _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/com.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_nodejs _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mex.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_nodejs _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_nodejs _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_nodejs _arbiter_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arbiter_gennodejs)
add_dependencies(arbiter_gennodejs arbiter_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arbiter_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter
)
_generate_msg_py(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter
)
_generate_msg_py(arbiter
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter
)

### Generating Services
_generate_srv_py(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/com.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter
)
_generate_srv_py(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/arbiter/msg/mex.msg;/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter
)
_generate_srv_py(arbiter
  "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter
)

### Generating Module File
_generate_module_py(arbiter
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arbiter_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arbiter_generate_messages arbiter_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/comVel.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_py _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/InteractionArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_py _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/com.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_py _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mex.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_py _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/msg/mexArray.msg" NAME_WE)
add_dependencies(arbiter_generate_messages_py _arbiter_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/arbiter/srv/getInteractionBuffer.srv" NAME_WE)
add_dependencies(arbiter_generate_messages_py _arbiter_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arbiter_genpy)
add_dependencies(arbiter_genpy arbiter_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arbiter_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arbiter
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(arbiter_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET rossini_task_db_generate_messages_cpp)
  add_dependencies(arbiter_generate_messages_cpp rossini_task_db_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arbiter
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(arbiter_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET rossini_task_db_generate_messages_eus)
  add_dependencies(arbiter_generate_messages_eus rossini_task_db_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arbiter
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(arbiter_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET rossini_task_db_generate_messages_lisp)
  add_dependencies(arbiter_generate_messages_lisp rossini_task_db_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arbiter
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(arbiter_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET rossini_task_db_generate_messages_nodejs)
  add_dependencies(arbiter_generate_messages_nodejs rossini_task_db_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arbiter
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(arbiter_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET rossini_task_db_generate_messages_py)
  add_dependencies(arbiter_generate_messages_py rossini_task_db_generate_messages_py)
endif()
