cmake_minimum_required(VERSION 3.10)

project(pcl_tutorial)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
        sensor_msgs
        pcl_ros)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        src
        ${catkin_INCLUDE_DIRS}
)

add_executable(lec1_usage lec1_usage.cpp)
target_link_libraries(lec1_usage ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(lec2_1_shared_ptr lec2_1_shared_ptr.cpp)
target_link_libraries(lec2_1_shared_ptr ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(lec2_2_ptr lec2_2_ptr.cpp)
target_link_libraries(lec2_2_ptr ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(lec2_3_ptr_in_class lec2_3_ptr_in_class.cpp)
target_link_libraries(lec2_3_ptr_in_class ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(lec3_tf lec3_transformation.cpp)
target_link_libraries(lec3_tf ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(lec4_voxel lec4_voxelization.cpp)
target_link_libraries(lec4_voxel ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(lec5_pass_through lec5_pass_through.cpp)
target_link_libraries(lec5_pass_through ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(lec6_sor lec6_sor.cpp)
target_link_libraries(lec6_sor ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_executable(lec7_normal lec7_normal.cpp)
target_link_libraries(lec7_normal ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})




