cmake_minimum_required(VERSION 2.8.3)
project(state_changer)
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
)

catkin_package(
	DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp")
add_executable(state_changer_node src/main.cpp)
target_link_libraries (state_changer_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
