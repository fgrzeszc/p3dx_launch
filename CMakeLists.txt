cmake_minimum_required(VERSION 2.8.3)
project(p3dx_launch)

find_package(catkin REQUIRED tf)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p3dx_launch
 CATKIN_DEPENDS roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########


## Declare a cpp executable
add_executable(tf_broadcaster src/tf_broadcaster.cpp)

target_link_libraries(tf_broadcaster
   ${catkin_LIBRARIES}
 )
