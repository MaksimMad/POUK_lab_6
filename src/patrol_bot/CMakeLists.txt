# cmake_minimum_required(VERSION 3.5)
# project(patrol_bot)

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# # Поиск зависимостей ROS 2
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

# # Создание исполняемого файла
# add_executable(patrol_bot_node
#   src/patrol_bot_node.cpp
# )
# target_include_directories(patrol_bot_node
#   PRIVATE
#     include
# )

# # Подключение зависимостей
# ament_target_dependencies(patrol_bot_node
#   rclcpp
#   geometry_msgs
#   nav2_msgs
#   tf2_ros
# )

# # Установка исполняемого файла
# install(TARGETS patrol_bot_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Установка launch файлов (если есть)
# # install(DIRECTORY
# #   launch
# #   DESTINATION share/${PROJECT_NAME}
# # )

# # Экспорт зависимостей
# ament_export_dependencies(
#   rclcpp
#   geometry_msgs
#   nav2_msgs
#   tf2_ros
# )

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(patrol_bot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(patrol_bot_node src/patrol_bot_node.cpp)
ament_target_dependencies(patrol_bot_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(TARGETS
  patrol_bot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()