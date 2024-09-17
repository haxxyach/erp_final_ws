# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rubber_cone_mission: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irubber_cone_mission:/home/unita/erp_final_ws/src/rubber_cone_mission/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rubber_cone_mission_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg" NAME_WE)
add_custom_target(_rubber_cone_mission_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rubber_cone_mission" "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rubber_cone_mission
  "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rubber_cone_mission
)

### Generating Services

### Generating Module File
_generate_module_cpp(rubber_cone_mission
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rubber_cone_mission
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rubber_cone_mission_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rubber_cone_mission_generate_messages rubber_cone_mission_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg" NAME_WE)
add_dependencies(rubber_cone_mission_generate_messages_cpp _rubber_cone_mission_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rubber_cone_mission_gencpp)
add_dependencies(rubber_cone_mission_gencpp rubber_cone_mission_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rubber_cone_mission_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rubber_cone_mission
  "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rubber_cone_mission
)

### Generating Services

### Generating Module File
_generate_module_eus(rubber_cone_mission
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rubber_cone_mission
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rubber_cone_mission_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rubber_cone_mission_generate_messages rubber_cone_mission_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg" NAME_WE)
add_dependencies(rubber_cone_mission_generate_messages_eus _rubber_cone_mission_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rubber_cone_mission_geneus)
add_dependencies(rubber_cone_mission_geneus rubber_cone_mission_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rubber_cone_mission_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rubber_cone_mission
  "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rubber_cone_mission
)

### Generating Services

### Generating Module File
_generate_module_lisp(rubber_cone_mission
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rubber_cone_mission
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rubber_cone_mission_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rubber_cone_mission_generate_messages rubber_cone_mission_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg" NAME_WE)
add_dependencies(rubber_cone_mission_generate_messages_lisp _rubber_cone_mission_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rubber_cone_mission_genlisp)
add_dependencies(rubber_cone_mission_genlisp rubber_cone_mission_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rubber_cone_mission_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rubber_cone_mission
  "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rubber_cone_mission
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rubber_cone_mission
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rubber_cone_mission
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rubber_cone_mission_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rubber_cone_mission_generate_messages rubber_cone_mission_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg" NAME_WE)
add_dependencies(rubber_cone_mission_generate_messages_nodejs _rubber_cone_mission_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rubber_cone_mission_gennodejs)
add_dependencies(rubber_cone_mission_gennodejs rubber_cone_mission_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rubber_cone_mission_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rubber_cone_mission
  "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rubber_cone_mission
)

### Generating Services

### Generating Module File
_generate_module_py(rubber_cone_mission
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rubber_cone_mission
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rubber_cone_mission_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rubber_cone_mission_generate_messages rubber_cone_mission_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/unita/erp_final_ws/src/rubber_cone_mission/msg/Fin.msg" NAME_WE)
add_dependencies(rubber_cone_mission_generate_messages_py _rubber_cone_mission_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rubber_cone_mission_genpy)
add_dependencies(rubber_cone_mission_genpy rubber_cone_mission_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rubber_cone_mission_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rubber_cone_mission)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rubber_cone_mission
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rubber_cone_mission_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rubber_cone_mission)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rubber_cone_mission
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rubber_cone_mission_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rubber_cone_mission)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rubber_cone_mission
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rubber_cone_mission_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rubber_cone_mission)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rubber_cone_mission
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rubber_cone_mission_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rubber_cone_mission)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rubber_cone_mission\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rubber_cone_mission
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rubber_cone_mission_generate_messages_py std_msgs_generate_messages_py)
endif()
