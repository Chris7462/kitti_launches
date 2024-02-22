cmake_minimum_required(VERSION 3.8)
project(kitti_launches)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
