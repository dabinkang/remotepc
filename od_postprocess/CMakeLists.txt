cmake_minimum_required(VERSION 3.0.2)
project(od_postprocess)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
set (wiringPi_include "/use/local/include")

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
  ObjectCount.msg
  Alert.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES od_postprocess
  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${wiringPi_include}
)

add_executable(alert src/alert.cpp)
add_dependencies(alert ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(alert ${catkin_LIBRARIES} wiringPi -lrt)
