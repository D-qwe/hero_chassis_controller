cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint
        controller_interface
        hardware_interface
        pluginlib
        geometry_msgs
        control_toolbox
        dynamic_reconfigure
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
)

## Generate dynamic reconfigure options (注意：必须放在 catkin_package 前面)
generate_dynamic_reconfigure_options(
        cfg/PidConfig.cfg
)

## catkin specific configuration
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        controller_interface
        hardware_interface
        pluginlib
        geometry_msgs
        control_toolbox
        dynamic_reconfigure
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs

)

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare the controller as a shared library
add_library(${PROJECT_NAME}
        src/HeroChassisController.cpp
)

## Add dependencies to exported targets
add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link the shared library against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

## Install the shared library
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install the header files
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

## Install the plugin description file
install(
        FILES hero_chassis_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
# Disabled for this assignment