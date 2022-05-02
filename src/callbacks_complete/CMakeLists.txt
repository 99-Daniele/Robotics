cmake_minimum_required(VERSION 2.8.3)
project(callbacks_complete)

## Find catkin and any catkin packages
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        tf2
        tf2_ros
        dynamic_reconfigure
)

add_message_files(
        FILES
        RPM.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

generate_dynamic_reconfigure_options(
        cfg/ApproximationParameters.cfg
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES subscriber_class
  CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(subscriber_class src/Subscriber.cpp)
add_dependencies(subscriber_class ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscriber_class ${catkin_LIBRARIES})

add_executable(sub_wheels src/main_sub.cpp)
add_dependencies(sub_wheels ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_wheels ${catkin_LIBRARIES} subscriber_class)


add_executable(Odometry src/Odometry.cpp)
add_dependencies(Odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(Odometry ${catkin_LIBRARIES} subscriber_class)


add_executable(velocity src/velocity.cpp)
add_dependencies(velocity ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity ${catkin_LIBRARIES} subscriber_class)
