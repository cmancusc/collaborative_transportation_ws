# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rossini_task_db: 2 messages, 2 services")

set(MSG_I_FLAGS "-Irossini_task_db:/home/federico/benzi_ws/src/rossini_task_db-master/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rossini_task_db_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg" NAME_WE)
add_custom_target(_rossini_task_db_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rossini_task_db" "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg" ""
)

get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv" NAME_WE)
add_custom_target(_rossini_task_db_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rossini_task_db" "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv" "rossini_task_db/Command:rossini_task_db/Interaction"
)

get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv" NAME_WE)
add_custom_target(_rossini_task_db_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rossini_task_db" "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv" "rossini_task_db/Command:rossini_task_db/Interaction"
)

get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg" NAME_WE)
add_custom_target(_rossini_task_db_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rossini_task_db" "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg" "rossini_task_db/Command"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rossini_task_db
)
_generate_msg_cpp(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rossini_task_db
)

### Generating Services
_generate_srv_cpp(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rossini_task_db
)
_generate_srv_cpp(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rossini_task_db
)

### Generating Module File
_generate_module_cpp(rossini_task_db
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rossini_task_db
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rossini_task_db_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rossini_task_db_generate_messages rossini_task_db_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_cpp _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_cpp _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_cpp _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_cpp _rossini_task_db_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rossini_task_db_gencpp)
add_dependencies(rossini_task_db_gencpp rossini_task_db_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rossini_task_db_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rossini_task_db
)
_generate_msg_eus(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rossini_task_db
)

### Generating Services
_generate_srv_eus(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rossini_task_db
)
_generate_srv_eus(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rossini_task_db
)

### Generating Module File
_generate_module_eus(rossini_task_db
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rossini_task_db
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rossini_task_db_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rossini_task_db_generate_messages rossini_task_db_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_eus _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_eus _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_eus _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_eus _rossini_task_db_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rossini_task_db_geneus)
add_dependencies(rossini_task_db_geneus rossini_task_db_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rossini_task_db_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rossini_task_db
)
_generate_msg_lisp(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rossini_task_db
)

### Generating Services
_generate_srv_lisp(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rossini_task_db
)
_generate_srv_lisp(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rossini_task_db
)

### Generating Module File
_generate_module_lisp(rossini_task_db
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rossini_task_db
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rossini_task_db_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rossini_task_db_generate_messages rossini_task_db_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_lisp _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_lisp _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_lisp _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_lisp _rossini_task_db_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rossini_task_db_genlisp)
add_dependencies(rossini_task_db_genlisp rossini_task_db_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rossini_task_db_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rossini_task_db
)
_generate_msg_nodejs(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rossini_task_db
)

### Generating Services
_generate_srv_nodejs(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rossini_task_db
)
_generate_srv_nodejs(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rossini_task_db
)

### Generating Module File
_generate_module_nodejs(rossini_task_db
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rossini_task_db
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rossini_task_db_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rossini_task_db_generate_messages rossini_task_db_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_nodejs _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_nodejs _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_nodejs _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_nodejs _rossini_task_db_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rossini_task_db_gennodejs)
add_dependencies(rossini_task_db_gennodejs rossini_task_db_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rossini_task_db_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rossini_task_db
)
_generate_msg_py(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rossini_task_db
)

### Generating Services
_generate_srv_py(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rossini_task_db
)
_generate_srv_py(rossini_task_db
  "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv"
  "${MSG_I_FLAGS}"
  "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg;/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rossini_task_db
)

### Generating Module File
_generate_module_py(rossini_task_db
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rossini_task_db
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rossini_task_db_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rossini_task_db_generate_messages rossini_task_db_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Command.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_py _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/SetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_py _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/srv/GetInteraction.srv" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_py _rossini_task_db_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/federico/benzi_ws/src/rossini_task_db-master/msg/Interaction.msg" NAME_WE)
add_dependencies(rossini_task_db_generate_messages_py _rossini_task_db_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rossini_task_db_genpy)
add_dependencies(rossini_task_db_genpy rossini_task_db_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rossini_task_db_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rossini_task_db)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rossini_task_db
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rossini_task_db_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rossini_task_db)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rossini_task_db
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rossini_task_db_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rossini_task_db)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rossini_task_db
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rossini_task_db_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rossini_task_db)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rossini_task_db
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rossini_task_db_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rossini_task_db)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rossini_task_db\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rossini_task_db
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rossini_task_db_generate_messages_py std_msgs_generate_messages_py)
endif()
