cmake_minimum_required(VERSION 2.8.3)
project(kuka_kr16_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/demo.launch)
  roslaunch_add_file_check(tests/moveit_planning_execution_rsi.xml)
endif()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
