# # cmake_minimum_required(VERSION 3.8)
# # project(navigation)

# # # Поиск базовых зависимостей ROS 2
# # find_package(ament_cmake REQUIRED)
# # find_package(rclcpp REQUIRED)
# # find_package(geometry_msgs REQUIRED)

# # # Поиск зависимостей Nav2 (должны быть установлены)
# # find_package(nav2_msgs REQUIRED)
# # find_package(nav2_common REQUIRED)
# # find_package(tf2_ros REQUIRED)

# # # Установка файлов
# # install(
# #   DIRECTORY config launch maps
# #   DESTINATION share/${PROJECT_NAME}
# # )

# # install(
# #   FILES navi.rviz
# #   DESTINATION share/${PROJECT_NAME}
# # )

# # ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(navigation)

# # Явно указываем тип пакета
# set(AMENT_PACKAGE_TYPE "ament_cmake")

# # Базовые зависимости
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)

# # Установка файлов
# install(
#   DIRECTORY launch config maps
#   DESTINATION share/${PROJECT_NAME}
# )

# install(
#   FILES navi.rviz
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()







cmake_minimum_required(VERSION 3.8)
project(my_navigation)

# Явно указываем тип пакета
set(AMENT_PACKAGE_TYPE "ament_cmake")

find_package(ament_cmake REQUIRED)

# Установка файлов (только если директории существуют)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(
    DIRECTORY maps/
    DESTINATION share/${PROJECT_NAME}/maps
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/navi.rviz")
  install(
    FILES navi.rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()