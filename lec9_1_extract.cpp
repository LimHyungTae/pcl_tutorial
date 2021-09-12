//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <ctime>
#include <string>

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

int main (int argc, char** argv) {
    /*
     * Load toy data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>);
    *src = *load_bin("/home/shapelim/catkin_ws/src/pcl_tutorial/materials/kitti00_000000.bin");

    /*
     * KDTree 선언
     */
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    int N = 100; // 차랑 주변 크기
    std::vector<int> idxes;
    std::vector<float> sqr_dists;

    kdtree.setInputCloud(src);
    pcl::PointXYZ query = src->points[100];

    cout<<query.x <<", "<< query.y<<", "<< query.z<<endl;

    kdtree.nearestKSearch(query, N, idxes, sqr_dists);

    for (const auto& idx: idxes){
        boundary->points.push_back(src->points[idx]);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::uint8_t r(255), g(15), b(15);
    for (const auto &basic_point: src->points) {
        pcl::PointXYZRGB point;
        point.x = basic_point.x;
        point.y = basic_point.y;
        point.z = basic_point.z;
        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                             static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        point.rgb = *reinterpret_cast<float *>(&rgb);
        src_color->points.push_back(point);
    }

    /*
     * Visualization!
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    r = 0;
    g = 255;
    b = 0;
    for (const auto &basic_point: boundary->points) {
        pcl::PointXYZRGB point;
        point.x = basic_point.x;
        point.y = basic_point.y;
        point.z = basic_point.z;
        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                             static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        point.rgb = *reinterpret_cast<float *>(&rgb);
        boundary_color->points.push_back(point);
    }

    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(src_color, "src");
    viewer.addPointCloud<pcl::PointXYZRGB>(boundary_color, "boundary");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}