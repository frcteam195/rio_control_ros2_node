cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME "rio_control_ros2_node")
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(boost_libraries_list
  date_time
  filesystem
)

set(THIRD_PARTY_LIB_FOLDER "/mnt/working/third_party_libs")

set(third_party_find_package_list
  ament_cmake
  Threads
)

set(ros_find_package_list
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  ck_ros2_base_msgs_node
  ck_utilities_ros2_node
  tf2
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
)

set(link_libraries
  Threads::Threads
  libzmq-static
)

add_custom_target(make_gen_inc_folder ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/generated/include/${PROJECT_NAME}
)

add_custom_target(make_gen_src_folder ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/generated/src
)

add_custom_target(
  generate_jinja_sources ALL
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generated/include/${PROJECT_NAME}/parameters.hpp ${CMAKE_CURRENT_SOURCE_DIR}/generated/src/parameters.cpp
)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/generated/include/${PROJECT_NAME}/parameters.hpp ${CMAKE_CURRENT_SOURCE_DIR}/generated/src/parameters.cpp
  COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/template/render_jinja.py --header_output ${CMAKE_CURRENT_SOURCE_DIR}/generated/include/${PROJECT_NAME}/parameters.hpp --header_template ${CMAKE_CURRENT_SOURCE_DIR}/template/parameters.hpp.j2
  --src_output ${CMAKE_CURRENT_SOURCE_DIR}/generated/src/parameters.cpp --src_template ${CMAKE_CURRENT_SOURCE_DIR}/template/parameters.cpp.j2
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/${PROJECT_NAME}.yaml ${CMAKE_CURRENT_SOURCE_DIR}/template/render_jinja.py ${CMAKE_CURRENT_SOURCE_DIR}/template/parameters.hpp.j2 ${CMAKE_CURRENT_SOURCE_DIR}/template/parameters.cpp.j2
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating header ${CMAKE_CURRENT_SOURCE_DIR}/generated/include/${PROJECT_NAME}/${PROJECT_NAME}.hpp"
)


include_directories(
 ${THIRD_PARTY_LIB_FOLDER}/ROS2_ProtoDef/out
)


file(GLOB_RECURSE executable_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB_RECURSE generated_sources CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/generated/src/*.cpp"
)

foreach(tdependency ${third_party_find_package_list})
  find_package(${tdependency} REQUIRED)
endforeach()

include(FindProtobuf)
find_package(Protobuf 3.21.12.0 EXACT REQUIRED PATHS /usr/local/lib)
include_directories(${PROTOBUF_INCLUDE_DIR})

find_package(ZeroMQ 4.3.4 EXACT REQUIRED PATHS /usr/local/lib)
include_directories(${ZeroMQ_INCLUDE_DIR})

foreach(rdependency ${ros_find_package_list})
  find_package(${rdependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS ${boost_libraries_list}) 

add_executable(${PROJECT_NAME} ${executable_sources} ${generated_sources})
add_dependencies(generate_jinja_sources make_gen_inc_folder make_gen_src_folder)
add_dependencies(${PROJECT_NAME} generate_jinja_sources)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
)

foreach(rtdependency ${ros_find_package_list})
  ament_target_dependencies(${PROJECT_NAME} ${rtdependency})
endforeach()

target_link_libraries(${PROJECT_NAME} ${link_libraries} ${Boost_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${THIRD_PARTY_LIB_FOLDER}/ROS2_ProtoDef/out/RobotComms.a)

target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARY})


target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
