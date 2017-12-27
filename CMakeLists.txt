cmake_minimum_required(VERSION 2.8.3)
project(active_constraints)

set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        custom_msgs
        tf_conversions
        )


catkin_package()

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_library(
        ACEnforceLib
        src/ActiveConstraintEnforcementMethods.cpp
        src/ACEnforcement.cpp
        ${catkin_LIBRARIES}
)

add_executable(
        dvrk_active_constraint_enforcement
        src/main_ac_enforcement.cpp)

target_link_libraries(
        dvrk_active_constraint_enforcement
        ACEnforceLib
        ${catkin_LIBRARIES}	)

add_executable(
        dvrk_active_constraint_geomerty_generation
        src/main_ac_geometry_generation.cpp
        src/ACGeometryGeneration.cpp
        src/ACGeometryGeneration.h)

target_link_libraries(
        dvrk_active_constraint_geomerty_generation
        ${catkin_LIBRARIES}	)
