# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "foundations_hw1: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/lunar/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/lunar/share/geometry_msgs/cmake/../msg;-Iturtlesim:/opt/ros/lunar/share/turtlesim/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(foundations_hw1_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv" NAME_WE)
add_custom_target(_foundations_hw1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "foundations_hw1" "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv" "geometry_msgs/Point:turtlesim/Pose"
)

get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv" NAME_WE)
add_custom_target(_foundations_hw1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "foundations_hw1" "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/lunar/share/turtlesim/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foundations_hw1
)
_generate_srv_cpp(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foundations_hw1
)

### Generating Module File
_generate_module_cpp(foundations_hw1
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foundations_hw1
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(foundations_hw1_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(foundations_hw1_generate_messages foundations_hw1_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_cpp _foundations_hw1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_cpp _foundations_hw1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foundations_hw1_gencpp)
add_dependencies(foundations_hw1_gencpp foundations_hw1_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foundations_hw1_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/lunar/share/turtlesim/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foundations_hw1
)
_generate_srv_eus(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foundations_hw1
)

### Generating Module File
_generate_module_eus(foundations_hw1
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foundations_hw1
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(foundations_hw1_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(foundations_hw1_generate_messages foundations_hw1_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_eus _foundations_hw1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_eus _foundations_hw1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foundations_hw1_geneus)
add_dependencies(foundations_hw1_geneus foundations_hw1_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foundations_hw1_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/lunar/share/turtlesim/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foundations_hw1
)
_generate_srv_lisp(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foundations_hw1
)

### Generating Module File
_generate_module_lisp(foundations_hw1
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foundations_hw1
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(foundations_hw1_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(foundations_hw1_generate_messages foundations_hw1_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_lisp _foundations_hw1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_lisp _foundations_hw1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foundations_hw1_genlisp)
add_dependencies(foundations_hw1_genlisp foundations_hw1_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foundations_hw1_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/lunar/share/turtlesim/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foundations_hw1
)
_generate_srv_nodejs(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foundations_hw1
)

### Generating Module File
_generate_module_nodejs(foundations_hw1
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foundations_hw1
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(foundations_hw1_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(foundations_hw1_generate_messages foundations_hw1_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_nodejs _foundations_hw1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_nodejs _foundations_hw1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foundations_hw1_gennodejs)
add_dependencies(foundations_hw1_gennodejs foundations_hw1_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foundations_hw1_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/lunar/share/turtlesim/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foundations_hw1
)
_generate_srv_py(foundations_hw1
  "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/lunar/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foundations_hw1
)

### Generating Module File
_generate_module_py(foundations_hw1
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foundations_hw1
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(foundations_hw1_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(foundations_hw1_generate_messages foundations_hw1_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Escape.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_py _foundations_hw1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cs4750/catkin_ws/src/foundations_hw1/srv/Reward.srv" NAME_WE)
add_dependencies(foundations_hw1_generate_messages_py _foundations_hw1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foundations_hw1_genpy)
add_dependencies(foundations_hw1_genpy foundations_hw1_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foundations_hw1_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foundations_hw1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foundations_hw1
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(foundations_hw1_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(foundations_hw1_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET turtlesim_generate_messages_cpp)
  add_dependencies(foundations_hw1_generate_messages_cpp turtlesim_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foundations_hw1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foundations_hw1
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(foundations_hw1_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(foundations_hw1_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET turtlesim_generate_messages_eus)
  add_dependencies(foundations_hw1_generate_messages_eus turtlesim_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foundations_hw1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foundations_hw1
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(foundations_hw1_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(foundations_hw1_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET turtlesim_generate_messages_lisp)
  add_dependencies(foundations_hw1_generate_messages_lisp turtlesim_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foundations_hw1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foundations_hw1
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(foundations_hw1_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(foundations_hw1_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET turtlesim_generate_messages_nodejs)
  add_dependencies(foundations_hw1_generate_messages_nodejs turtlesim_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foundations_hw1)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foundations_hw1\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foundations_hw1
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foundations_hw1
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foundations_hw1/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(foundations_hw1_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(foundations_hw1_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET turtlesim_generate_messages_py)
  add_dependencies(foundations_hw1_generate_messages_py turtlesim_generate_messages_py)
endif()
