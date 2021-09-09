//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ>& pc_dst, double var_voxel_size){

    static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pc_src);
    voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
    voxel_filter.filter(pc_dst);
}

//Input: pcl::PointCloud source, pc_src
//Output: voxelized pcl::PointCloud, pc_voxelized
int main(int argc, char **argv){

    pcl::PointCloud<pcl::PointXYZ> pc_src;
    int num_pts = 100;
    pcl::PointXYZ point_xyz;
    for (int k=0; k < num_pts ; ++k){
        float x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 5;
        float y = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 5;
        float z = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 5;
        point_xyz.x = x;     point_xyz.y = y;     point_xyz.z = z;
        pc_src.push_back(point_xyz);
    }


    /*
     * Main
     */
    pcl::PointCloud<pcl::PointXYZ> pc_voxelized;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    double var_voxelsize = 0.05;

    *ptr_src = pc_src;
    voxel_filter.setInputCloud(ptr_src);
    voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
    voxel_filter.filter(*ptr_filtered);

    pc_voxelized = *ptr_filtered;

    // filter()함수에 ptr을 넣지 않고 직접적으로 pcl::PointCloud<pcl::PointXYZ>로 받아도 된다.
    voxelize(ptr_src, pc_voxelized, var_voxelsize);

    return 0;

}