# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cf_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-Icf_msgs:/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cf_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg" NAME_WE)
add_custom_target(_cf_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cf_msgs" "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg" NAME_WE)
add_custom_target(_cf_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cf_msgs" "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg" NAME_WE)
add_custom_target(_cf_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cf_msgs" "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg" NAME_WE)
add_custom_target(_cf_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cf_msgs" "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg" NAME_WE)
add_custom_target(_cf_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cf_msgs" "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg" NAME_WE)
add_custom_target(_cf_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cf_msgs" "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs
)
_generate_msg_cpp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs
)
_generate_msg_cpp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs
)
_generate_msg_cpp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs
)
_generate_msg_cpp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs
)
_generate_msg_cpp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(cf_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cf_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cf_msgs_generate_messages cf_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_cpp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_cpp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_cpp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_cpp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_cpp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_cpp _cf_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cf_msgs_gencpp)
add_dependencies(cf_msgs_gencpp cf_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cf_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs
)
_generate_msg_eus(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs
)
_generate_msg_eus(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs
)
_generate_msg_eus(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs
)
_generate_msg_eus(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs
)
_generate_msg_eus(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(cf_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cf_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cf_msgs_generate_messages cf_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_eus _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_eus _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_eus _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_eus _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_eus _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_eus _cf_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cf_msgs_geneus)
add_dependencies(cf_msgs_geneus cf_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cf_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs
)
_generate_msg_lisp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs
)
_generate_msg_lisp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs
)
_generate_msg_lisp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs
)
_generate_msg_lisp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs
)
_generate_msg_lisp(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(cf_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cf_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cf_msgs_generate_messages cf_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_lisp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_lisp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_lisp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_lisp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_lisp _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_lisp _cf_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cf_msgs_genlisp)
add_dependencies(cf_msgs_genlisp cf_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cf_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs
)
_generate_msg_nodejs(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs
)
_generate_msg_nodejs(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs
)
_generate_msg_nodejs(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs
)
_generate_msg_nodejs(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs
)
_generate_msg_nodejs(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(cf_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cf_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cf_msgs_generate_messages cf_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_nodejs _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_nodejs _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_nodejs _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_nodejs _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_nodejs _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_nodejs _cf_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cf_msgs_gennodejs)
add_dependencies(cf_msgs_gennodejs cf_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cf_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs
)
_generate_msg_py(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs
)
_generate_msg_py(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs
)
_generate_msg_py(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs
)
_generate_msg_py(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs
)
_generate_msg_py(cf_msgs
  "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(cf_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cf_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cf_msgs_generate_messages cf_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Gyro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_py _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Flow.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_py _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tof.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_py _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Tdoa.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_py _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Accel.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_py _cf_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/abhi/repositories/dsl/dsl__projects__uwbDataset/ros_ws/src/cf_msgs/msg/Baro.msg" NAME_WE)
add_dependencies(cf_msgs_generate_messages_py _cf_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cf_msgs_genpy)
add_dependencies(cf_msgs_genpy cf_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cf_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cf_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cf_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cf_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cf_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cf_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cf_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cf_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cf_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cf_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cf_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
