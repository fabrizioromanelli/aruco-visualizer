cmake_minimum_required(VERSION 3.5)
project(aruco-visualizer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(LIBS 
-lboost_system
)

find_package(OpenCV 4.5.1 QUIET)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4.5.1 not found.")
endif()

add_executable(${PROJECT_NAME} src/aruco-visualizer.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs cv_bridge)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${LIBS})

install(DIRECTORY
    DESTINATION share/${PROJECT_NAME}
    )

install(TARGETS
${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
