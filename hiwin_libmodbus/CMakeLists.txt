cmake_minimum_required(VERSION 3.8)
project(hiwin_libmodbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define path of libmodbus folder here.
find_path(LIBMODBUS_PATH
  NAMES "README.md"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../libmodbus/"
)
message("Libmodbus path dir = ${LIBMODBUS_PATH}")
# Run autogen.sh if config.h.in does not exist.
find_path(CONFIGURE_IN_PATH
  NAMES "config.h.in"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../libmodbus/"
)
if(NOT CONFIGURE_IN_PATH)
  execute_process(
    COMMAND "${LIBMODBUS_PATH}/autogen.sh"
    WORKING_DIRECTORY ${LIBMODBUS_PATH}
    RESULT_VARIABLE AUTOGEN_RESULT
    OUTPUT_VARIABLE AUTOGEN_OUTPUT
    ERROR_VARIABLE AUTOGEN_ERROR
  )
  if(AUTOGEN_RESULT)
    message("Libmodbus autogen RESULT = ${AUTOGEN_RESULT}")
  endif()
  if(AUTOGEN_OUTPUT)
    message("Libmodbus autogen OUTPUT = ${AUTOGEN_OUTPUT}")
  endif()
  if(AUTOGEN_ERROR)
    message("Libmodbus autogen ERROR = ${AUTOGEN_ERROR}")
  endif()
endif()

# Run configure if config.h does not exist.
find_path(CONFIGURE_H_PATH
  NAMES "config.h"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../libmodbus/"
)
if(NOT CONFIGURE_H_PATH)
  execute_process(
    COMMAND ${LIBMODBUS_PATH}/configure
    WORKING_DIRECTORY ${LIBMODBUS_PATH}
    RESULT_VARIABLE CONFIGURE_RESULT
    OUTPUT_VARIABLE CONFIGURE_OUTPUT
    ERROR_VARIABLE CONFIGURE_ERROR
  )
  if(CONFIGURE_RESULT)
    message("Libmodbus configure RESULT = ${CONFIGURE_RESULT}")
  endif()
  if(CONFIGURE_OUTPUT)
    message("Libmodbus configure OUTPUT = ${CONFIGURE_OUTPUT}")
  endif()
  if(CONFIGURE_ERROR)
    message("Libmodbus configure ERROR = ${CONFIGURE_ERROR}")
  endif()
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(hiwin_action_interfaces REQUIRED)
find_package(rclcpp)
find_package(rclcpp_action)
find_package(rclcpp_components) 


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(LIBMODBUS_CORE_FILES
  ${LIBMODBUS_PATH}/src/modbus-data.c
  ${LIBMODBUS_PATH}/src/modbus-rtu.c
  ${LIBMODBUS_PATH}/src/modbus-tcp.c
  ${LIBMODBUS_PATH}/src/modbus.c
)

add_library(${PROJECT_NAME} SHARED
  src/hiwin_libmodbus.cpp
  ${LIBMODBUS_CORE_FILES}
)

target_compile_features(${PROJECT_NAME} 
  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${LIBMODBUS_PATH}/src>
  $<BUILD_INTERFACE:${LIBMODBUS_PATH}>
  $<INSTALL_INTERFACE:include>)

add_library(${PROJECT_NAME}_ctype_wrapper SHARED
  src/hiwin_libmodbus_ctype_wrapper.cpp)
target_compile_features(${PROJECT_NAME}_ctype_wrapper
  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}_ctype_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}_ctype_wrapper
  ${PROJECT_NAME}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE 
  "HIWIN_LIBMODBUS_BUILDING_LIBRARY")
target_compile_definitions(${PROJECT_NAME}_ctype_wrapper PRIVATE
  "HIWIN_LIBMODBUS_CTYPE_WRAPPER_BUILDING_LIBRARY")


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS ${PROJECT_NAME}_ctype_wrapper
  EXPORT export_${PROJECT_NAME}_ctype_wrapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(action_server SHARED
  src/hiwinmodbus_action_server.cpp)

target_compile_features(${PROJECT_NAME}_ctype_wrapper
  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(action_server PUBLIC
  action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(action_server
  ${PROJECT_NAME}
)

#target_include_directories(action_server PRIVATE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 # $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "hiwin_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)



add_executable(hiwin_libmodbus_node src/hiwin_libmodbus_node.cpp)
add_executable(hiwinmodbus_action_server src/hiwinmodbus_action_server.cpp)
ament_target_dependencies(hiwinmodbus_action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "hiwin_action_interfaces")
target_link_libraries(hiwinmodbus_action_server
  ${PROJECT_NAME}
)

target_include_directories(hiwin_libmodbus_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hiwin_libmodbus_node hiwin_libmodbus)

install(TARGETS hiwin_libmodbus_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/Hiwin_API_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  hiwin_libmodbus
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
