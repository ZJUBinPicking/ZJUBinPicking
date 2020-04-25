cmake_minimum_required(VERSION 2.8.3)
project(template_match)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PCL_DIR "/usr/local/include/pcl-1.10/pcl") 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  cv_bridge
sensor_msgs

)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp
                   pcl_conversions
                   pcl_ros
                    pcl_ros 
  )



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
   ${template_match_SOURCE_DIR} include/
)
add_executable(template_match src/main.cpp)
target_link_libraries(template_match ${catkin_LIBRARIES})
  target_link_libraries (template_match libvtkCommonCore-8.2.so.1 libvtkRenderingOpenGL2-8.2.so.1)
catkin_package()