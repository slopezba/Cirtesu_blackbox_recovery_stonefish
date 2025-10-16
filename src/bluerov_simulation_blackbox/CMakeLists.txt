cmake_minimum_required(VERSION 3.0.2)
project(bluerov_simulation_blackbox)

find_package(catkin REQUIRED COMPONENTS roscpp
    rosparam
    cola2_msgs
    actionlib
    controller_manager
    control_msgs
)

catkin_package(INCLUDE_DIRS
    CATKIN_DEPENDS roscpp
    rosparam
    DEPENDS
)

# Check if found controller_manager
include_directories(
    ${catkin_INCLUDE_DIRS}
)

if(NOT controller_manager_FOUND)
    message(WARNING "controller_manager not found. predefined_positions will not be built")
endif()

if(NOT actionlib_FOUND)
    message(WARNING "actionlib not found. predefined_positions will not be built")
endif()


if(controller_manager_FOUND AND actionlib_FOUND)
    add_executable(predefined_positions src/predefined_positions.cpp)
    target_link_libraries(predefined_positions ${catkin_LIBRARIES})
endif()
