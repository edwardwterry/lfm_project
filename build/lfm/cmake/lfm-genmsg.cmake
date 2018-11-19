# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lfm: 6 messages, 0 services")

set(MSG_I_FLAGS "-Ilfm:/home/et/Documents/lfm_ws/src/lfm/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lfm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg" NAME_WE)
add_custom_target(_lfm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lfm" "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg" "lfm/Block:std_msgs/Header"
)

get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg" NAME_WE)
add_custom_target(_lfm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lfm" "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg" ""
)

get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg" NAME_WE)
add_custom_target(_lfm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lfm" "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg" ""
)

get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg" NAME_WE)
add_custom_target(_lfm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lfm" "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg" "std_msgs/Header:geometry_msgs/Quaternion:lfm/AprilTagDetection:geometry_msgs/Point:geometry_msgs/PoseWithCovariance:geometry_msgs/PoseWithCovarianceStamped:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg" NAME_WE)
add_custom_target(_lfm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lfm" "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/PoseWithCovariance:geometry_msgs/PoseWithCovarianceStamped:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg" NAME_WE)
add_custom_target(_lfm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lfm" "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm
)
_generate_msg_cpp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm
)
_generate_msg_cpp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm
)
_generate_msg_cpp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm
)
_generate_msg_cpp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm
)
_generate_msg_cpp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm
)

### Generating Services

### Generating Module File
_generate_module_cpp(lfm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lfm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lfm_generate_messages lfm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg" NAME_WE)
add_dependencies(lfm_generate_messages_cpp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg" NAME_WE)
add_dependencies(lfm_generate_messages_cpp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg" NAME_WE)
add_dependencies(lfm_generate_messages_cpp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg" NAME_WE)
add_dependencies(lfm_generate_messages_cpp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(lfm_generate_messages_cpp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg" NAME_WE)
add_dependencies(lfm_generate_messages_cpp _lfm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lfm_gencpp)
add_dependencies(lfm_gencpp lfm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lfm_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm
)
_generate_msg_eus(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm
)
_generate_msg_eus(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm
)
_generate_msg_eus(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm
)
_generate_msg_eus(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm
)
_generate_msg_eus(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm
)

### Generating Services

### Generating Module File
_generate_module_eus(lfm
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lfm_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lfm_generate_messages lfm_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg" NAME_WE)
add_dependencies(lfm_generate_messages_eus _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg" NAME_WE)
add_dependencies(lfm_generate_messages_eus _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg" NAME_WE)
add_dependencies(lfm_generate_messages_eus _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg" NAME_WE)
add_dependencies(lfm_generate_messages_eus _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(lfm_generate_messages_eus _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg" NAME_WE)
add_dependencies(lfm_generate_messages_eus _lfm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lfm_geneus)
add_dependencies(lfm_geneus lfm_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lfm_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm
)
_generate_msg_lisp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm
)
_generate_msg_lisp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm
)
_generate_msg_lisp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm
)
_generate_msg_lisp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm
)
_generate_msg_lisp(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm
)

### Generating Services

### Generating Module File
_generate_module_lisp(lfm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lfm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lfm_generate_messages lfm_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg" NAME_WE)
add_dependencies(lfm_generate_messages_lisp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg" NAME_WE)
add_dependencies(lfm_generate_messages_lisp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg" NAME_WE)
add_dependencies(lfm_generate_messages_lisp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg" NAME_WE)
add_dependencies(lfm_generate_messages_lisp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(lfm_generate_messages_lisp _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg" NAME_WE)
add_dependencies(lfm_generate_messages_lisp _lfm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lfm_genlisp)
add_dependencies(lfm_genlisp lfm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lfm_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm
)
_generate_msg_nodejs(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm
)
_generate_msg_nodejs(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm
)
_generate_msg_nodejs(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm
)
_generate_msg_nodejs(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm
)
_generate_msg_nodejs(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm
)

### Generating Services

### Generating Module File
_generate_module_nodejs(lfm
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lfm_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lfm_generate_messages lfm_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg" NAME_WE)
add_dependencies(lfm_generate_messages_nodejs _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg" NAME_WE)
add_dependencies(lfm_generate_messages_nodejs _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg" NAME_WE)
add_dependencies(lfm_generate_messages_nodejs _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg" NAME_WE)
add_dependencies(lfm_generate_messages_nodejs _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(lfm_generate_messages_nodejs _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg" NAME_WE)
add_dependencies(lfm_generate_messages_nodejs _lfm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lfm_gennodejs)
add_dependencies(lfm_gennodejs lfm_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lfm_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm
)
_generate_msg_py(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm
)
_generate_msg_py(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm
)
_generate_msg_py(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm
)
_generate_msg_py(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm
)
_generate_msg_py(lfm
  "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm
)

### Generating Services

### Generating Module File
_generate_module_py(lfm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lfm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lfm_generate_messages lfm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/State.msg" NAME_WE)
add_dependencies(lfm_generate_messages_py _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Links.msg" NAME_WE)
add_dependencies(lfm_generate_messages_py _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Action.msg" NAME_WE)
add_dependencies(lfm_generate_messages_py _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetectionArray.msg" NAME_WE)
add_dependencies(lfm_generate_messages_py _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(lfm_generate_messages_py _lfm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/et/Documents/lfm_ws/src/lfm/msg/Block.msg" NAME_WE)
add_dependencies(lfm_generate_messages_py _lfm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lfm_genpy)
add_dependencies(lfm_genpy lfm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lfm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lfm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lfm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(lfm_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(lfm_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lfm
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lfm_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(lfm_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(lfm_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lfm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lfm_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(lfm_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(lfm_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lfm
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lfm_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(lfm_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(lfm_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lfm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lfm_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(lfm_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(lfm_generate_messages_py sensor_msgs_generate_messages_py)
endif()
