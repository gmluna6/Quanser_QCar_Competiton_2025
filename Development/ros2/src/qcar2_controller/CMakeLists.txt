cmake_minimum_required(VERSION 3.8)
project(qcar2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)

find_package(qcar2_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Need to define _NVIDIA macro since the Quanser SDK has different
# functionalities depending on which platform it's being used.
add_compile_definitions(_NVIDIA)

# The header 'cv_bridge/cv_bridge.hpp' was added in version 3.3.0. For older
# cv_bridge versions, we have to use the header 'cv_bridge/cv_bridge.h'.
if(${cv_bridge_VERSION} VERSION_GREATER_EQUAL "3.3.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCV_BRDIGE_HAS_HPP")
endif()

ament_python_install_package(${PROJECT_NAME})

include_directories(include)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    ${PROJECT_NAME}/xbox_command.py
    ${PROJECT_NAME}/motor_mux.py
    ${PROJECT_NAME}/stop_sign_handler.py
    ${PROJECT_NAME}/stop_sign2.py
    ${PROJECT_NAME}/redlight_detection.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
