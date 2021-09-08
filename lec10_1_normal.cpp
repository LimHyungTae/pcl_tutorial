//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>


template<typename T>
void voxelize(const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, pcl::PointCloud<T> &dst, double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(srcPtr);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(dst);
}
template <typename KDTree>

void calc_normal(KDTree& kdtree,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                 int K,
                 pcl::PointCloud<pcl::Normal>::Ptr& cloud_normal){

    //Initialization
    clock_t start, middle, end;
    start = clock();
    kdtree.setInputCloud(input_cloud);
    middle = clock();
    pcl::Normal normal_tmp;
    std::vector<int> idxNano(K);
    std::vector<float> distNano(K);
    int num_pc = input_cloud->points.size();

    for (int i = 0; i < num_pc; ++i) {
        pcl::PointXYZ &query = input_cloud->points[i];
        kdtree.nearestKSearch(query, K, idxNano, distNano);
        // Reserve() is important for speed up!
        pcl::PointCloud<pcl::PointXYZ>::Ptr NN(new pcl::PointCloud<pcl::PointXYZ>);
        NN->reserve(num_pc);
        for (int tgt_idx:idxNano) {
            NN->points.emplace_back(input_cloud->points[tgt_idx]);
        }
        Eigen::Matrix3f cov;
        Eigen::Vector4f mean;
        pcl::computeMeanAndCovarianceMatrix(*NN, cov, mean);
        // Singular Value Decomposition: SVD
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
        // use the least singular vector as normal
        Eigen::MatrixXf normal = (svd.matrixU().col(2));
        normal_tmp.normal_x = normal(0, 0);
        normal_tmp.normal_y = normal(1, 0);
        normal_tmp.normal_z = normal(2, 0);
        cloud_normal->points.emplace_back(normal_tmp);
    }
    end = clock();
    float time_setting = float(middle-start) /  CLOCKS_PER_SEC;
    float time_taken = float(end-start) /  CLOCKS_PER_SEC;
    std::cout<<"Setting: "<<time_setting<<" s..."<<std::endl;
    std::cout<<"Searching: "<<time_taken - time_setting<<" s..."<<std::endl;
    std::cout<<"Total: "<<time_taken<<" s..."<<std::endl;

}
int main (int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            "/home/shapelim/catkin_ws/src/pcl_tutorial/materials/naverlabs_vel16.pcd", *raw_cloud) ==
        -1) {
        PCL_ERROR ("Couldn't read source pcd file! \n");
        return (-1);
    }
    Eigen::Matrix4f lidar0ToBody;
    lidar0ToBody << 0, -1, 0, 0,
            -1, 0, -0, 0,
            0, 0, -1, 0.4,
            0, 0, 0, 1;
    pcl::transformPointCloud(*raw_cloud, *cloud, lidar0ToBody);
    voxelize(cloud, *cloud_voxel, 0.05);
    *cloud = *cloud_voxel;
    std::cout<<cloud->points.size()<<std::endl;
    pcl::KdTreeFLANN<pcl::PointXYZ> PCLFLANN_kdtree;
    int K = 20;
    std::vector<int> idxNano(K);
    std::vector<float> distNano(K);

    int num_pc = cloud->points.size();
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal_dummy(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    cloud_normal->points.reserve(num_pc);


    std::cout<<"PCL: "<<std::endl;
    calc_normal(PCLFLANN_kdtree, cloud, K, cloud_normal);


    std::cout << "Total num: " << num_pc << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::uint8_t r(255), g(15), b(15);
    for (const auto &basic_point: cloud->points) {
        pcl::PointXYZRGB point;
        point.x = basic_point.x;
        point.y = basic_point.y;
        point.z = basic_point.z;
        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                             static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        point.rgb = *reinterpret_cast<float *>(&rgb);
        cloud_color->points.push_back(point);
    }

    std::cout<<"[Debug]: "<<cloud_color->points.size()<< " <-> "<<cloud_normal->points.size()<<std::endl;
    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_color);

    int level = 7; // the larger, more normal vectors are displayed
    float scale = 0.7; // size of arrow
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_color, cloud_normal, level, scale,
                                                               "Simple Cloud Viewer", 0);
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}