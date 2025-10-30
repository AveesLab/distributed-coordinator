cmake_minimum_required(VERSION 3.5)
project(coordinator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB VIMBA_CPP_SOURCE_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/include/VimbaCPP/Source/*.cpp"
)

add_executable(coordinator_node
  src/main.cpp
  src/coordinator.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
  ${VIMBA_CPP_SOURCE_FILES}
)

target_include_directories(coordinator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  /opt/Vimba_6_0/VimbaCPP/Include
  /opt/Vimba_6_0/VimbaC/Include
)

target_link_libraries(coordinator_node
  /opt/Vimba_6_0/VimbaCPP/DynamicLib/arm_64bit/libVimbaCPP.so
  /opt/Vimba_6_0/VimbaC/DynamicLib/arm_64bit/libVimbaC.so
  ${OpenCV_LIBS}
)

ament_target_dependencies(coordinator_node
  rclcpp
  std_msgs
  sensor_msgs
  camera_info_manager
  diagnostic_msgs
  image_transport
  cv_bridge
)

install(TARGETS coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 남겨도 무방 (런타임에 인터페이스 사용)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

