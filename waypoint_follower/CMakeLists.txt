cmake_minimum_required(VERSION 3.0.2)
project(waypoint_follower)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  geometry_msgs
  ackermann_msgs
  roscpp
  waypoint_maker
  serial
  delivery_mission
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_follower
  CATKIN_DEPENDS nav_msgs std_msgs geometry_msgs ackermann_msgs roscpp waypoint_maker serial delivery_mission
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ./include/waypoint_follower
  ${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
   src/new_main.cpp
   src/new_process.cpp
   src/new_pure_pursuit.cpp      
   src/delivery_new_pure_pursuit.cpp
)

add_executable(${PROJECT_NAME}_node 
   src/new_main.cpp
   src/new_process.cpp
   src/new_pure_pursuit.cpp  
   src/delivery_new_pure_pursuit.cpp    
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)


