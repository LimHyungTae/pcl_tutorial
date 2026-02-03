cmake_minimum_required(VERSION 3.10)

project(pcl_tutorial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost 1.58 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        src
)

file(COPY ${CMAKE_SOURCE_DIR}/materials/
        DESTINATION ${CMAKE_BINARY_DIR}/auxiliary/
        FILES_MATCHING
        PATTERN *.bin)

add_executable(lec00_usage lec00_usage.cpp)
target_link_libraries(lec00_usage ${PCL_LIBRARIES})

add_executable(lec01_1_shared_ptr lec01_1_shared_ptr.cpp)
target_link_libraries(lec01_1_shared_ptr ${PCL_LIBRARIES})

add_executable(lec01_2_ptr lec01_2_ptr.cpp)
target_link_libraries(lec01_2_ptr ${PCL_LIBRARIES})

add_executable(lec01_3_ptr_in_class lec01_3_ptr_in_class.cpp)
target_link_libraries(lec01_3_ptr_in_class ${PCL_LIBRARIES})

add_executable(lec03_tf lec03_transformation.cpp)
target_link_libraries(lec03_tf ${PCL_LIBRARIES})

add_executable(lec04_visualization lec04_visualization.cpp)
target_link_libraries(lec04_visualization ${PCL_LIBRARIES})

add_executable(lec05_voxel lec05_voxelization.cpp)
target_link_libraries(lec05_voxel ${PCL_LIBRARIES})

add_executable(lec06_pass_through lec06_pass_through.cpp)
target_link_libraries(lec06_pass_through ${PCL_LIBRARIES})

add_executable(lec07_sor lec07_sor.cpp)
target_link_libraries(lec07_sor ${PCL_LIBRARIES})

add_executable(lec08_radius_search lec08_radius_search.cpp)
target_link_libraries(lec08_radius_search ${PCL_LIBRARIES})

add_executable(lec09_knn lec09_knn.cpp)
target_link_libraries(lec09_knn ${PCL_LIBRARIES})

add_executable(lec10_1_normal lec10_1_normal.cpp)
target_link_libraries(lec10_1_normal ${PCL_LIBRARIES})

add_executable(lec10_2_normal_corner lec10_2_normal_corner.cpp)
target_link_libraries(lec10_2_normal_corner ${PCL_LIBRARIES})

add_executable(lec11_icp lec11_icp.cpp)
target_link_libraries(lec11_icp ${PCL_LIBRARIES})

add_executable(lec12_gicp lec12_gicp.cpp)
target_link_libraries(lec12_gicp ${PCL_LIBRARIES})
