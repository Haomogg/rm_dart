cmake_minimum_required(VERSION 3.0.2)
project(rm_dart)

find_package(catkin REQUIRED COMPONENTS
        nodelet
        roscpp
        rm_msgs
        dynamic_reconfigure
        cv_bridge
        pluginlib
        image_transport
        rm_msgs
        rm_common
)

find_package(OpenCV REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
        cfg/Dart.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
        nodelet
        roscpp
        dynamic_reconfigure
        cv_bridge
        image_transport
        rm_msgs
        rm_common
#  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/rm_dart_aim.cpp)

target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)