cmake_minimum_required(VERSION 3.8)
project(agri_greenhouse_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/generate_tomato_field.py scripts/extract_tomato_fruit.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES LICENSE NOTICE README.md DESTINATION share/${PROJECT_NAME})

ament_package()
