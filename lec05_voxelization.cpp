//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_bin(const string &filename) {
    FILE*file                     = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "Error: failed to load " << filename << std::endl;
        return nullptr;
    }
    std::vector<float> buffer(1000000);
    size_t             num_points =
            fread(reinterpret_cast<char*>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(num_points);

    for (int i = 0; i < num_points; i++) {
        auto &pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        // pt.intensity = buffer[i * 4 + 3];
    }

    return cloud;
}

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

    int N              = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int         i = 0; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ>& pc_dst, double var_voxel_size){

    static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pc_src);
    voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
    voxel_filter.filter(pc_dst);
}

//Input: pcl::PointCloud source, pc_src
//Output: voxelized pcl::PointCloud, pc_voxelized
int main(int argc, char **argv){
    /*
     * Load toy data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    *src = *load_bin("/home/shapelim/git/pcl_tutorial/materials/kitti00_000000.bin");

    /**
     * Main
     */
    pcl::PointCloud<pcl::PointXYZ> pc_voxelized;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    float voxelsize = 0.2;

    voxel_filter.setInputCloud(src);
    voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize);
    voxel_filter.filter(*ptr_filtered);

    pc_voxelized = *ptr_filtered;

    // filter()함수에 ptr을 넣지 않고 직접적으로 pcl::PointCloud<pcl::PointXYZ>로 받아도 된다.
    voxelize(src, pc_voxelized, voxelsize);
    /*******************************************/

    /**
     * 결과 visualization 하기
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Point cloud XYZ에 RGB 칼라 추가하기
    colorize(*src, *src_colored, {255, 0, 0});
    colorize(*ptr_filtered, *vox_colored, {0, 255, 0});

    pcl::visualization::PCLVisualizer viewer1("Raw");
    pcl::visualization::PCLVisualizer viewer2("voxel");

    viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
    viewer2.addPointCloud<pcl::PointXYZRGB>(vox_colored, "vox_green");

    while (!viewer1.wasStopped() && !viewer2.wasStopped()) {
        viewer1.spinOnce();
        viewer2.spinOnce();
    }


    return 0;

}