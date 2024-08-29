cmake_minimum_required(VERSION 3.8)
project(trainee_launch)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

install(PROGRAMS
  scripts/map_save.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  scripts
)
