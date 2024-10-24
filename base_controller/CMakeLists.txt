cmake_minimum_required(VERSION 3.10)
project(base_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    tf2_geometry_msgs
)

## Find Eigen3 for matrix and vector computations
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  	${catkin_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS
    roscpp
  DEPENDS
 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${EIGEN_INCLUDE_DIRS}
)


add_executable(			${PROJECT_NAME} src/base_controller.cpp)
target_link_libraries(	${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)