cmake_minimum_required(VERSION 3.0.2)
project(easy_bag_recorder)

add_compile_options(-std=c++17 -Wall -Wextra -Werror) # TODO add -pedantic
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_CXX_CLANG_TIDY clang-tidy;--quiet)
set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++17;--error-exitcode=1)
set(CMAKE_LINK_WHAT_YOU_USE TRUE)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag std_msgs actionlib actionlib_msgs topic_tools)

add_action_files(
  DIRECTORY action
  FILES
  Record.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rosbag actionlib actionlib_msgs topic_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

